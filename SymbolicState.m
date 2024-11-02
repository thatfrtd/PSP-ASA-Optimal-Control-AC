syms mass L I g;
r = sym("r", [2;1]);
v = sym("v", [2;1]);
theta = sym("theta", 1);
w = sym("w", 1);
x = [r;v;theta;thetadot];

thrust = sym("thrust", 1);
gimbal = sym("gimbal", 1);
u = [thrust;gimbal];

rdot = v;
thetadot = w;
wdot = -L*thrust*sin(gimbal)/I;

totalTheta = theta+gimbal;
rotationMatrix = [cos(totalTheta) sin(totalTheta); sin(totalTheta) cos(totalTheta)]
T_e = rotationMatrix * [thrust; 0]
vdot = T_e/mass - [0;g]

xdot = [rdot; vdot; thetadot; wdot]

j_a = jacobian(xdot, x);
j_b = jacobian(xdot, u);

xdot_func = matlabFunction(xdot);
j_a_func = matlabFunction(j_a);
j_b_func = matlabFunction(j_b);

save("3DoF\dynamics3dof.mat", "xdot_func", "j_a_func", "j_b_func")