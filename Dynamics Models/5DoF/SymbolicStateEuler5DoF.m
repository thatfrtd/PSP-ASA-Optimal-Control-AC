% constants (same as 3DoF)
g = 9.81; % [m / s2]

syms mass L I max_thrust; 
c = [mass; L; I];

% states

r = sym("r", [3;1]);
v = sym("v", [3,1]);
theta = sym("theta", [3;1]);
w = sym("w", [2;1]);

x = [r; v; theta; w];

% controls

norm_T = sym("T", [3;1]);

u = [norm_T];

T = norm_T * max_thrust;

% calculate theta dot
b_inverse =  (1/sin(theta(2))) .* [0 sin(theta(3)) cos(theta(3))
     0 sin(theta(2))*cos(theta(3)) -sin(theta(2))*sin(theta(3))
     sin(theta(2)) -cos(theta(2))*sin(theta(3)) -cos(theta(2))*cos(theta(3))];
zero_w = [0; w];
thetadot = b_inverse * zero_w;

% r dot
r_dot = v;

Rxtheta1 = [1 0 0; 0 cos(theta(1)) sin(theta(1)); 0 -sin(theta(1)) cos(theta(1))];
Rxtheta2 = [cos(theta(2)) 0 -sin(theta(2)); 0 1 0; sin(theta(2)) 0 cos(theta(2))];
Rxtheta3 = [1 0 0; 0 cos(theta(3)) sin(theta(3)); 0 -sin(theta(3)) cos(theta(3))];
C_be = Rxtheta3 * Rxtheta2 * Rxtheta1;
C_eb = C_be.';
T_e = C_eb * T;
% v dot
v_dot = T_e / mass - [0; 0; g];

% w dot
M = cross([-L; 0; 0], T);
w_dot = M / I;

x_dot = [r_dot; v_dot; thetadot; w_dot(2:3)];
j_a = jacobian(x_dot, x);
j_b = jacobian(x_dot, u);

vars = [x; u; mass; L; I; max_thrust];

% Create equations of motion function for optimizer
matlabFunction(x_dot,"File","Dynamics Models/5DoF/SymDynamicsEuler5DoF","Vars",vars);

% Create equations of motion block for Simulink model
open("EoM_Euler5DoF.slx")
matlabFunctionBlock('EoM_Euler5DoF/SymDynamicsEuler5DoF',x_dot,'Vars',vars);
close_system("EoM_Euler5DoF.slx", 1)

% Create Jacobian functions for Kalman filter
matlabFunction(j_a,"File","Dynamics Models/5DoF/SymXJacobianEuler5DoF","Vars",vars);
matlabFunction(j_b,"File","Dynamics Models/5DoF/SymUJacobianEuler5DoF","Vars",vars);

%open("JacobianX_5DoF.slx")
%matlabFunctionBlock("JacobianX_5DoF/SymXJacobianEuler5DoF",j_a,"Vars",vars);
%close_system("JacobianX_5DoF.slx", 1)


% xdot_func5dof = matlabFunction(state);
% j_a_func5dof = matlabFunction(j_a);
% j_b_func5dof = matlabFunction(j_b);
% 
% save("dynamics5dof.mat", "xdot_func5dof", "j_a_func5dof", "j_b_func5dof")
% save("SymDynamics5DoF.m", "xdot_func5dof")
% save("SymUJacobian5DoF.m", "j_b_func5dof")
% save("SymXJacobian5DoF.m", "j_a_func5dof")