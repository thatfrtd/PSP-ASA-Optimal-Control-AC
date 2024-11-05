x_initial = [0, 1000, 0, -80, pi/2, 0];

steps = 400;

x_guess = zeros([steps, 6]);
u_guess = zeros([steps, 2]);

vehicle = load("Astra2.mat").vehicle;

[u_opt, x_opt] = trajOptimizer3([x_initial(1), x_initial(3), x_initial(2), x_initial(4), pi/2 - x_initial(5), x_initial(6)])

%[u_opt, x_opt] = TrajOptimizer(x_initial, x_guess, u_guess, vehicle)

%[u_opt, x_opt] = TrajOptimizer_codable_mex(x_initial, x_opt, u_opt, vehicle)