vehicle = load("Astra2").vehicle;
dof = DoF.euler5DoF;
tspan = [0 5];
x0 = [0,0,100,0,10,0,0,-pi/2,0,0,0.1];
[t,y] = ode45(@(t,y) Dynamics_nDoF(y, [0,0,0], vehicle, dof), tspan, x0);

%% position
plot(t,y(:,1),t,y(:,2),t,y(:,3))
legend("x","y","z")

%% velocity
plot(t,y(:,4),t,y(:,5),t,y(:,6))
legend("vx","vy","vz")

%% orientation
plot(t,y(:,7),t,y(:,8),t,y(:,9))
legend("e1_1","e2_2","e1_3")

%% angular velocity
plot(t,y(:,10),t,y(:,11))
legend("wy","wz")