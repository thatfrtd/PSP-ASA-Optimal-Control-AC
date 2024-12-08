% constants (same as 3DoF)
syms mass L I g; 
c = [g; mass; L; I]

% states

r = sym("r", [3;1])
v = sym("v", [3,1])
theta = sym("theta", [3;1])
w = sym("w", [2;1])

x = [r; v; theta; w]

% controls

T = sym("T", [3;1])

u = [T]

% calculate theta dot
b_inverse =  (1/sin(theta(2))) .* [0 sin(theta(3)) cos(theta(3))
     0 sin(theta(2))*cos(theta(3)) -sin(theta(2))*sin(theta(3))
     sin(theta(2)) -cos(theta(2))*sin(theta(3)) -cos(theta(2))*cos(theta(3))]
zero_w = [0; w]
thetadot = b_inverse * zero_w

% x dot
x_dot = v

Rxtheta1 = [1 0 0; 0 cos(theta(1)) -sin(theta(1)); 0 sin(theta(1)) cos(theta(1))]
Rxtheta2 = [cos(theta(2)) 0 sin(theta(2)); 0 1 0; -sin(theta(2)) 0 cos(theta(2))]
Rxtheta3 = [1 0 0; 0 cos(theta(3)) -sin(theta(3)); 0 sin(theta(3)) cos(theta(3))]
C_be = Rxtheta1 * Rxtheta2 * Rxtheta3
C_eb = C_be.'
T_e = C_be * T
% v dot
v_dot = T_e / mass - [0;0;g]

state = [x_dot; v_dot; thetadot; L*T(2)/I; -L*T(3)/I]
j_a = jacobian(state, x)
j_b = jacobian(state, u)


xdot_func5dof = matlabFunction(state);
j_a_func5dof = matlabFunction(j_a);
j_b_func5dof = matlabFunction(j_b);

save("dynamics5dof.mat", "xdot_func5dof", "j_a_func5dof", "j_b_func5dof")
save("SymDynamics5DoF.m", "xdot_func5dof")
save("SymUJacobian5DoF.m", "j_b_func5dof")
save("SymXJacobian5DoF.m", "j_a_func5dof")