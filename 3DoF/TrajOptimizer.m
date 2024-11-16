function [u_opt, x_opt] = TrajOptimizer(x_initial, x_guess, u_guess, vehicle, show_plots)
    %input current state and vehicle information, output control vector and potential state
    % TODO
    % - Add free final time through having time be normalized and adding a
    % parameter for time dialation (the time scaling factor)
    %       - Hopefully should greatly increase the robustness of the MPC
    % - Generalize to higher degree of freedom models if possible
    % - Account for delay from computation time when setting initial
    % condition
    % - Add glideslope constraint
    arguments
        x_initial
        x_guess
        u_guess
        vehicle
        show_plots = false
    end

    % Define the optimization problem
    opti = casadi.Opti();
    
    % Set the number of steps and the timestep (dt)
    steps = 400;
    t_step = 0.04;
    max_iter = 400;

    % Number of iterations to run (test warm starting)
    iterations = 1;
    
    % Generate the array of state and control vectors
    
    % States: x, y, x_dot, y_dot, theta, theta_dot
    x = opti.variable(steps, 6);  % steps x 6 matrix
    % Controls: thrust (percent), thrust_angle (rad)
    u = opti.variable(steps, 2);
    
    % Initial and final conditions
    % 0, 0, 1000, -80, -pi/2, 0
    x_final = [0, 0, 0, 0, deg2rad(90), 0];
    opti.subject_to(x(steps, :) == x_final); % Final state
    
    % Cost function to minimize effort and angular velocity
    
    cost = sum(u(:,1).^2) + sum(u(:,2).^2) + 2 * sum(x(:,6).^2) + t_step * steps;
    opti.minimize(cost);
    
    %constraints
    for i = 1:(steps-1)
        % Current state
        x_current = x(i, :)';
        u_current = u(i, :)';
        
        % Define the state derivatives
        x_dot = Dynamics3DoF(x_current, u_current, vehicle);
        
        % Euler integration for dynamics constraints
        x_next = x_current + x_dot * t_step;
        
        % Impose the dynamics constraint
        opti.subject_to(x(i+1, :)' == x_next);
    end
    
    % Thrust percentage bounds
    opti.subject_to(u(:,1) >= vehicle.min_thrust / vehicle.max_thrust);
    opti.subject_to(u(:,1) <= 1);

    % State constraints
    opti.subject_to(x(:,2) >= 0); % Make the rocket not go through the ground - replace with glideslope constraint
    
    % Thrust angle bounds
    opti.subject_to(u(:,2) >= -vehicle.max_gimbal);
    opti.subject_to(u(:,2) <= vehicle.max_gimbal);
    
    % Solver options
    %p_opts = struct('expand',true,'error_on_fail',false,'verbose',false);
    %s_opts = struct('max_iter',max_iter);
    %opti.solver('ipopt', p_opts, s_opts);
    opti.solver('ipopt');

    jump = 20;

    for i = 1:iterations
        if i > 1
            opti.set_value(p, x_opt(jump + 1, :));
        else
            p = opti.parameter(1, 6);
            opti.set_value(p, [x_initial(1), x_initial(2), x_initial(3), x_initial(4), x_initial(5), x_initial(6)]);
            opti.subject_to(x(1, :) == p); % Initial state
        end
    
        % Initial guess 
        if i > 1
            opti.set_initial(u, [u_opt((jump + 1):end,:); repmat(u_opt(end,:), jump, 1)]);
            opti.set_initial(x, [x_opt((jump + 1):end,:); repmat(x_opt(end,:), jump, 1)]);
        elseif ~isempty(x_guess) & ~isempty(u_guess)
            opti.set_initial(x, x_guess);
            opti.set_initial(u, u_guess);
        else
            [x_guess, u_guess] = guess_3DoF(x_initial, x_final, steps, t_step, vehicle);
            opti.set_initial(x, x_guess);
            opti.set_initial(u, u_guess);
        end
        
        % Solve the optimization problem
        sol = opti.solve();

        u_opt = sol.value(u);
        x_opt = sol.value(x);
        cost_opt = sol.value(cost);

        if i == 1
            % Initializing Dual Variables
            lam_g0 = sol.value(opti.lam_g);
            opti.set_initial(opti.lam_g, lam_g0);
        end
    end

    %Plots
    if show_plots
        time = t_step * (0:(steps-1));

        figure('Name', 'Optimization Results', 'NumberTitle', 'off', 'Color', 'w');
    
        % Plot state variables
        subplot(2,1,1);
        hold on;
        plot(time, x_opt(:,1), 'LineWidth', 1.5, 'DisplayName', 'x (Position)');
        plot(time, x_opt(:,2), 'LineWidth', 1.5, 'DisplayName', 'y (Position)');
        plot(time, x_opt(:,3), 'LineWidth', 1.5, 'DisplayName', 'x\_dot (Velocity)');
        plot(time, x_opt(:,4), 'LineWidth', 1.5, 'DisplayName', 'y\_dot (Velocity)');
        plot(time, x_opt(:,5), 'LineWidth', 1.5, 'DisplayName', 'theta (Angle)');
        plot(time, x_opt(:,6), 'LineWidth', 1.5, 'DisplayName', 'theta\_dot (Angular Velocity)');
        hold off;
        legend('Location', 'best');
        xlabel('Time [s]');
        ylabel('State Values');
        title('State Variables');
        grid on;
    
        % control inputs
        subplot(2,1,2); 
        hold on;
        plot(time, u_opt(:,1), 'LineWidth', 1.5, 'DisplayName', 'Thrust %');
        plot(time, u_opt(:,2), 'LineWidth', 1.5, 'DisplayName', 'Thrust Angle (rad)');
        hold off;
        legend('Location', 'best');
        xlabel('Time [s]');
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
        disp(u_opt)

        fprintf("Cost: %.2f, t_f: %.2f, t_step: %.4f, Impulse: %.0f \n", cost_opt, duration, t_step, t_step * sum(u_opt(:,1)))
    end
end