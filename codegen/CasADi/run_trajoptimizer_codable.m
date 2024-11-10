x_initial = [0, 1000, 0, -80, 0, 0];

steps = 400;

x_guess = zeros([steps, 6]);
u_guess = zeros([steps, 2]);

vehicle = load("Astra2.mat").vehicle;

% Interpreted pass (side effect of performing CasADi codegen)
TrajOptimizer_codable(x_initial, x_guess, u_guess, vehicle);

% Perform Matlab Coder step (links to CasADi generated code)
codegen TrajOptimizer_codable -args {zeros(1, 6), zeros(size(x_guess)), zeros(size(u_guess)), vehicle}

% Call the geneated mex function
TrajOptimizer_codable_mex(x_initial, x_guess, u_guess, vehicle)