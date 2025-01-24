function [x_guess, u_guess] = guess_nDoF(x_0, x_f, steps, t_step, vehicle, dof)
%STATE_GUESS Summary of this function goes here
%   Detailed explanation goes here

g = 9.81; % [m/s2]

% Control guess
T_guess = 0.4; % [N] Initial thrust guess
u_guess = [T_guess, zeros([1, dof.nu - 1])] .* ones([steps, dof.nu]);

% Parameter guess
v_0 = x_0(dof.iv);
v_f = x_f(dof.iv);
%tf_guess = norm(v_f-v_0 + sqrt(2 * [0; g] * x_0(2)), 2)/(g - T_guess * vehicle.max_thrust/vehicle.m);

% State guess
x_guess = zeros([steps, dof.nx]);

% r_guess
x_guess(:, dof.ir) = straight_line_interpolate(x_0(dof.ir), x_f(dof.ir), steps);

% v_guess
v_cst = (x_f(dof.ir) - x_0(dof.ir)) / (steps * t_step);
x_guess(:, dof.iv) = repmat(v_cst, steps, 1);

% CHANGE TO SPHERICALLY INTERPOLATE ANGLES AND ANGULAR VELOCITIES!!
% θ_guess 
x_guess(:, dof.itheta) = straight_line_interpolate(x_0(dof.itheta), x_f(dof.itheta), steps);

% ω_guess
w_cst = (x_0(dof.itheta) - x_f(dof.itheta)) / (steps * t_step);

x_guess(:, dof.iw) = repmat(w_cst(1:numel(dof.iw)), steps, 1);

end

