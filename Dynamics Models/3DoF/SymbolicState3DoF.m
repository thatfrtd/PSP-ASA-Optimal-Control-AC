%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PSP ASA - AC TrajOpt
% 3 Degree of Freedom Symbolic Equations of Motion Script
% Author: Lucas Mohler, Travis Hastreiter 
% Created On: 2 November, 2024
% Description: Creates symbolic equations of motion (EoM) for 3DoF planar 
% rocket landing problem. Creates function file and Simulink block for the 
% EoMs and function files for the Jacobians.
% Most Recent Change: Travis Hastreiter 5 November, 2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

g = 9.81;

syms mass L I max_thrust;
r = sym("r", [2;1]);
v = sym("v", [2;1]);
theta = sym("theta", 1);
w = sym("w", 1);
x = [r;v;theta;w];

norm_thrust = sym("thrust", 1); % Normalized thrust
gimbal = sym("gimbal", 1);
u = [norm_thrust;gimbal];

thrust = norm_thrust * max_thrust;

rdot = v;
thetadot = w;
wdot = -L*thrust*sin(gimbal)/I;

totalTheta = theta+gimbal;
rotationMatrix = [cos(totalTheta) sin(totalTheta); sin(totalTheta) cos(totalTheta)];
T_e = rotationMatrix * [thrust; 0];
vdot = T_e/mass - [0; g];

xdot = [rdot; vdot; thetadot; wdot];

j_a = jacobian(xdot, x);
j_b = jacobian(xdot, u);

vars = [x; u; mass; L; I; max_thrust];

% Create equations of motion function for optimizer
matlabFunction(xdot,"File","3DoF/SymDynamics3DoF","Vars",vars);

% Create equations of motion block for Simulink model
open("EoM_3DoF.slx")
matlabFunctionBlock('EoM_3DoF/SymDynamics3DoF',xdot,'Vars',vars)
close_system("EoM_3DoF.slx", 1)

% Create Jacobian functions for Kalman filter
matlabFunction(j_a,"File","3DoF/SymXJacobian3DoF","Vars",vars);
matlabFunction(j_b,"File","3DoF/SymUJacobian3DoF","Vars",vars);

open("JacobianX_3DoF.slx")
matlabFunctionBlock("JacobianX_3DoF/SymXJacobian3DoF",j_a,"Vars",vars);
close_system("JacobianX_3DoF.slx", 1)