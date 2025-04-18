out = load("CasADi_freetf_out_0p3delay.mat").out;

%%
simout = Get_Sim_Output(out);

num_timestep = size(simout.utraj, 1);
num_optim = size(simout.utraj, 3);
op_time = 0.3;
dtime = 0.3;
offset = op_time * (0:num_optim-1);
scaled = (dtime' * (0:num_timestep-1))';
solution_time = offset + scaled;
min_sol_time = min(solution_time,[],"all");
max_sol_time = max(solution_time,[],"all");
min_dtime = min(dtime,[],"all");
min_optime = min(op_time,[],"all");
k = linspace(min_sol_time, max_sol_time, num_optim);

q = linspace(0, max(simout.time), num_optim);
c = min_sol_time:op_time:max_sol_time;

[X, Y] = meshgrid(q,c);


scaled_values = zeros(numel(c), numel(q)); %2nd axis
for i = 1:length(q) 
    scaled_values(:, i) = interp1(solution_time(:,i), simout.xtraj(:,2,i), c, 'linear', 0); 
end

scaled_first_axis = zeros(numel(c),numel(q)); %first axis
for i = 1:length(q) 
    scaled_first_axis(:, i) = interp1(solution_time(:,i), simout.xtraj(:,1,i), c, 'linear', 0); 
end

scaled_third_axis = zeros(numel(c), numel(q)); %third axis
for i = 1:length(q) 
    scaled_third_axis(:, i) = interp1(solution_time(:,i), simout.xtraj(:,3,i), c, 'linear', 0); 
end

scaled_fourth_axis = zeros(numel(c), numel(q)); %fourth axis
for i = 1:length(q) 
    scaled_fourth_axis(:, i) = interp1(solution_time(:,i), simout.xtraj(:,4,i), c, 'linear', 0); 
end 

scaled_fifth_axis = zeros(numel(c), numel(q)); %fifth axis
for i = 1:length(q) 
    scaled_fifth_axis(:, i) = interp1(solution_time(:,i), simout.xtraj(:,5,i), c, 'linear', 0); 
end

scaled_throttle_values = zeros(numel(c), numel(q)); %2nd axis
for i = 1:length(q) 
    scaled_throttle_values(:, i) = interp1(solution_time(:,i), simout.utraj(:,1,i), c, 'linear', 0); 
end

scaled_gimbal_values = zeros(numel(c), numel(q)); %2nd axis
for i = 1:length(q) 
    scaled_gimbal_values(:, i) = interp1(solution_time(:,i), simout.utraj(:,2,i), c, 'linear', 0); 
end

% plots

% height
figure
waterfall(X, Y, scaled_values);
xlabel("Simulation Time [s]")
ylabel("Solution Time [s]")
zlabel("Height [m]")
title("Height Trajectory vs Time [Fixed]")


% crossrange trajectory
 %   axis labels may be changed
figure
waterfall(X, Y, scaled_first_axis);
ylabel("Solution Time [s]")
xlabel("Simulation Time [s]")
zlabel("Crossrange [m]")
title("Crossrange Trajectory vs Time [Fixed]")

% throttle
figure
waterfall(X, Y, scaled_throttle_values * 100);
xlabel("Simulation Time [s]")
ylabel("Solution Time [s]")
zlabel("Throtte [%]")
title("Throttle Trajectory vs Time [Fixed]")

% Horizontal Velocity 
    % what is the difference between scaling it as Y, X?
    % also this one might be busted :(
figure
waterfall(Y, X, scaled_third_axis);
ylabel("Simulation Time [s]")
xlabel("Solution Time")
zlabel("Horizontal Velocity [m/s]")
title("Horizontal Velocity Trajectory vs Time [Fixed]")

%Gimbal
figure
waterfall(X,Y, rad2deg(scaled_gimbal_values));
xlabel("Simulation Time [s]")
ylabel("Solution Time [s]")
zlabel("Gimbal [deg]")
title("Gimbal Trajectory vs Time [Fixed]")

%Orientation
    %axis may need to be changed
figure
waterfall(Y, X, rad2deg(scaled_fifth_axis));
ylabel("Simulation Time [s]")
xlabel("Solution Time [s]")
zlabel("Orientation [deg]")
title("Orientation Trajectory vs Time [Fixed]")

%Vertical Velocity
    %this one also might be busted
figure
waterfall(Y, X, scaled_fourth_axis);
xlabel("Solution Time [s]")
ylabel("Simulation Time [s]")
zlabel("Vertical Velocity [m/s]")
title("Vertical Velocity Trajectory vs Time [Fixed]")

view([61.4 29.4])