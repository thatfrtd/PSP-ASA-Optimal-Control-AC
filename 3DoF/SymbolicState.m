g = 9.81;

syms mass L I;
r = sym("r", [2;1]);
v = sym("v", [2;1]);
theta = sym("theta", 1);
w = sym("w", 1);
x = [r;v;theta;w];

thrust = sym("thrust", 1);
gimbal = sym("gimbal", 1);
u = [thrust;gimbal];

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

matlabFunction(xdot,"File","3DoF/SymDynamics3DoF","Vars",[x; u; mass; L; I]);
matlabFunction(j_a,"File","3DoF/SymXJacobian3DoF","Vars",[x; u; mass; L; I]);
matlabFunction(j_b,"File","3DoF/SymUJacobian3DoF","Vars",[x; u; mass; L; I]);