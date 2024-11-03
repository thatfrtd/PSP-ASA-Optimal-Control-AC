function [u_opt, x_opt] = TrajOptimizer_codable(x_initial, x_guess, u_guess, vehicle)
    %input current state and vehicle information, output control vector and potential state
    % TODO
    % - Add free final time through having time be normalized and adding a
    % parameter for time dialation (the time scaling factor)
    %       - Hopefully should greatly increase the robustness of the MPC
    % - Generalize to higher degree of freedom models if possible
    % - Account for delay from computation time when setting initial
    % condition
    % - Add glideslope constraint

    % Any pre-processing using pure Matlab operations can go here
    
    m = vehicle.m;
    L = vehicle.L;
    I = vehicle.I;
    max_gimbal = vehicle.max_gimbal;
    min_thrust = vehicle.min_thrust;
    max_thrust = vehicle.max_thrust;

    % Set the number of steps and the timestep (dt)
    steps = 400;
    t_step = 0.04;
    max_iter = 400;

    % Make sure data-types and sizes are known
    u_opt_sol = zeros([steps, 2]);
    x_opt_sol = zeros([steps, 6]);
    
    % Anything CasADi related goes here
    if coder.target('MATLAB')
        % Normal CasADi usage + CasADi codegen
        % Define the optimization problem
        opti = casadi.Opti();
    
        % Generate the array of state and control vectors
        
        % States: x, y, x_dot, y_dot, theta, theta_dot
        x = opti.variable(steps, 6);  % steps x 6 matrix
        % Controls: thrust (percent), thrust_angle (rad)
        u = opti.variable(steps, 2);
        
        % Initial and final conditions
        % 0, 0, 1000, -80, -pi/2, 0
        
        opti.subject_to(x(steps, :) == [0, 0, 0, 0, deg2rad(90), 0]); % Final state
        
        % Cost function to minimize effort and angular velocity
        
        cost = sum(u(:,1).^2) + sum(u(:,2).^2) + 2 * sum(x(:,6).^2);
        opti.minimize(cost);
        
        %constraints
        for i = 1:(steps-1)
            % Current state
            x_current = x(i, :)';
            u_current = u(i, :)';
            
            % Define the state derivatives
            x_dot = Dynamics3DoF(x_current, u_current .* [max_thrust; 1], vehicle)';
            
            % Euler integration for dynamics constraints
            x_next = x_current + x_dot * t_step;
            
            % Impose the dynamics constraint
            opti.subject_to(x(i+1, :)' == x_next);
        end
        
        % Thrust percentage bounds
        opti.subject_to(u(:,1) >= min_thrust / max_thrust);
        opti.subject_to(u(:,1) <= 1);
    
        % State constraints
        opti.subject_to(x(:,2) >= 0); % Make the rocket not go through the ground - replace with glideslope constraint
        
        % Thrust angle bounds
        opti.subject_to(u(:,2) >= -max_gimbal);
        opti.subject_to(u(:,2) <= max_gimbal);
        
        % Solver options
        p_opts = struct('expand',true,'error_on_fail',false,'verbose',false);
        s_opts = struct('max_iter',max_iter);
        opti.solver('ipopt', p_opts, s_opts);
    
        x_i = opti.parameter(1, 6);
        opti.set_value(x_i, [x_initial(1), x_initial(2), x_initial(3), x_initial(4), x_initial(5), x_initial(6)]);
        opti.subject_to(x(1, :) == x_i); % Initial state
    
        % Initial guess 
        %u_g = opti.parameter(steps, 2);
        %x_g = opti.parameter(steps, 6);
        %opti.set_value(u_g, u_guess);
        %opti.set_value(x_g, x_guess);
        %opti.set_initial(u, u_g);
        %opti.set_initial(x, x_g);
        opti.set_initial(u, u_guess);
        opti.set_initial(x, x_guess);

        % Codegen via a CasADi Function
        F = opti.to_function('Traj3DoF',{x_i},{u, x});
        %[u_opt_sol, x_opt_sol] = F(x_initial, x_guess, u_guess);
        [u_opt_sol, x_opt_sol] = F(x_initial);

        % Generate C code
        F.generate('Traj3DoF.c',struct('unroll_args',true,'with_header',true));
    
        % Generate meta-data
        config = struct;
        config.sz_arg = F.sz_arg();
        config.sz_res = F.sz_res();
        config.sz_iw = F.sz_iw();
        config.sz_w = F.sz_w();
        config.include_path = casadi.GlobalOptions.getCasadiIncludePath;
        config.path = casadi.GlobalOptions.getCasadiPath;
        if ismac
          config.link_library_suffix = '.dylib';
          config.link_library_prefix = 'lib';
        elseif isunix
          config.link_library_suffix = '.so';
          config.link_library_prefix = 'lib';
        elseif ispc
          config.link_library_suffix = '.lib';
          config.link_library_prefix = '';
        end
        save('Traj3DoF_config.mat','-struct','config');
    else
        % This gets executed when Matlab Coder is parsing the file
        % Hooks up Matlab Coder with CasADi generated C code
    
        % Connect .c and .h file
        coder.cinclude('Traj3DoF.h');
        coder.updateBuildInfo('addSourceFiles','Traj3DoF.c');
        
        % Set link and include path
        config = coder.load('Traj3DoF_config.mat');
        coder.updateBuildInfo('addIncludePaths',config.include_path)
        
        % Link with IPOPT
        coder.updateBuildInfo('addLinkObjects', [config.link_library_prefix 'ipopt' config.link_library_suffix], config.path, '', true, true);
    
        % Setting up working space
        arg = coder.opaque('const casadi_real*');
        res = coder.opaque('casadi_real*');
        iw = coder.opaque('casadi_int');
        w = coder.opaque('casadi_real');
    
        arg = coder.nullcopy(cast(zeros(config.sz_arg,1),'like',arg));
        res = coder.nullcopy(cast(zeros(config.sz_res,1),'like',res));
        iw  = coder.nullcopy(cast(zeros(config.sz_iw,1),'like',iw));
        w   = coder.nullcopy(cast(zeros(config.sz_w,1),'like',w));
    
        mem = int32(0);
        flag= int32(0);
        mem = coder.ceval('Traj3DoF_checkout');

        % Call the generated CasADi code
        flag=coder.ceval('Traj3DoF_unrolled',...
            coder.rref(x_initial), coder.rref(x_guess), coder.rref(u_guess),... % Adapt to as many inputs arguments as your CasADi Function has
            coder.wref(u_opt_sol), coder.wref(x_opt_sol), ... % Adapt to as many outputs as your CasADi Function has
            arg, res, iw, w, mem); % 
        coder.ceval('Traj3DoF_release', mem);
    end

    % Any post-processing using pure Matlab operations can go here

    x_opt = x_opt_sol;
    u_opt = u_opt_sol;
    %{
    x_opt = zeros(size(x_opt_sol));
    u_opt = zeros(size(u_opt_sol));
    x_opt(:, :) = x_opt_sol(:, :);
    u_opt(:, :) = u_opt_sol(:, :);

    %Plots

    figure('Name', 'Optimization Results', 'NumberTitle', 'off', 'Color', 'w');

    % Plot state variables
    subplot(2,1,1);
    hold on;
    plot(x_opt(:,1), 'LineWidth', 1.5, 'DisplayName', 'x (Position)');
    plot(x_opt(:,2), 'LineWidth', 1.5, 'DisplayName', 'y (Position)');
    plot(x_opt(:,3), 'LineWidth', 1.5, 'DisplayName', 'x\_dot (Velocity)');
    plot(x_opt(:,4), 'LineWidth', 1.5, 'DisplayName', 'y\_dot (Velocity)');
    plot(x_opt(:,5), 'LineWidth', 1.5, 'DisplayName', 'theta (Angle)');
    plot(x_opt(:,6), 'LineWidth', 1.5, 'DisplayName', 'theta\_dot (Angular Velocity)');
    hold off;
    legend('Location', 'best');
    xlabel('Time Step');
    ylabel('State Values');
    title('State Variables');
    grid on;

    % control inputs
    subplot(2,1,2); 
    hold on;
    plot(u_opt(:,1), 'LineWidth', 1.5, 'DisplayName', 'Thrust %');
    plot(u_opt(:,2), 'LineWidth', 1.5, 'DisplayName', 'Thrust Angle (rad)');
    hold off;
    legend('Location', 'best');
    xlabel('Time Step');
    ylabel('Control Inputs');
    title('Control Inputs');
    grid on;

    % times

    final_time_step = t_step;
    duration = t_step * steps;

    fprintf('Final Time Step: %.4f seconds\n', final_time_step);
    fprintf('Total Duration: %.4f seconds\n', duration);

    fprintf("Complete state matrix: \n______________________________\n")
    disp(x_opt)
    fprintf("Complete control matrix: \n______________________________\n")
    %}
end