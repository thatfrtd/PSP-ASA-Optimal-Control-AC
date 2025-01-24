%% Extract output data
simout = Get_Sim_Output(out);

%% Process data
theta = simout.x(:, 7:9)';
C_eb = permute(angle2dcm(theta(1,:), theta(2, :), theta(3, :), "XYX"),[2,1,3]);
% o = ones(size(theta(1, :)'));
% z = zeros(size(theta(1, :)'));
% Rxtheta1 = permute(reshape([o z z z cos(theta(1,:))' sin(theta(1,:))' z -sin(theta(1,:))' cos(theta(1,:))'], [], 3, 3), [3, 2, 1]);
% Rxtheta2 = permute(reshape([cos(theta(2,:))' z -sin(theta(2,:))' z o z sin(theta(2,:))' z cos(theta(2,:))'], [], 3, 3), [3, 2, 1]);
% Rxtheta3 = permute(reshape([o z z z cos(theta(3,:))' sin(theta(3,:))' z -sin(theta(3,:))' cos(theta(3,:))'], [], 3, 3), [3, 2, 1]);
% C_eb = pagemtimes(Rxtheta1, pagemtimes(Rxtheta2, Rxtheta3));

b_x = pagemtimes(C_eb, [1; 0; 0]);
b_y = pagemtimes(C_eb, [0; 1; 0]);
b_z = pagemtimes(C_eb, [0; 0; 1]);

T = reshape(simout.u(:, 1:3)', 3, 1, []);
T_e = pagemtimes(C_eb, T);

gimbal_r = acos(squeeze(pagemtimes([1 0 0], T_e)) ./ squeeze(vecnorm(T_e, 2, 1)))
gimbal_theta = squeeze(atan2(T(3, 1, :), T(2, 1, :))) + pi

velocity = vecnorm(simout.x(:, 4:6), 2, 2)

z_max = max(simout.x(:, 3)) * 1.1
x_lim = [min(simout.x(:, 1)) - z_max / 10, max(simout.x(:, 1)) + z_max / 10]
y_lim = [min(simout.x(:, 2)) - z_max / 10, max(simout.x(:, 2)) + z_max / 10]

xz_angle = asin(vecnorm(cross([squeeze(b_x(1, :)); zeros([1, numel(simout.time)]); squeeze(b_x(3, :))], repmat([1; 0; 0], 1, numel(simout.time)))) ./ vecnorm([squeeze(b_x(1, :)); zeros([1, numel(simout.time)]); squeeze(b_x(3, :))]))
yz_angle = asin(vecnorm(cross([zeros([1, numel(simout.time)]); squeeze(b_x(2, :)); squeeze(b_x(3, :))], repmat([1; 0; 0], 1, numel(simout.time)))) ./ vecnorm([zeros([1, numel(simout.time)]); squeeze(b_x(2, :)); squeeze(b_x(3, :))]))

%% XZ Projection
loops = numel(simout.time);

for j = 1:loops
    quiver([simout.x(j, 1)], [simout.x(j, 3)], [b_x(1, 1, j)] .* z_max ./ 10, [b_x(3, 1, j)] .* z_max ./ 10, Color='b'); hold on
    quiver([simout.x(j, 1)], [simout.x(j, 3)], [b_y(1, 1, j)] .* z_max ./ 10, [b_y(3, 1, j)] .* z_max ./ 10, Color='g'); hold on
    quiver([simout.x(j, 1)], [simout.x(j, 3)], [b_z(1, 1, j)] .* z_max ./ 10, [b_z(3, 1, j)] .* z_max ./ 10, Color='cyan'); hold on
    plot([simout.x(j, 1), simout.x(j, 1) - squeeze(T_e(1, 1, j)) * z_max ./ 10], [simout.x(j, 3), simout.x(j, 3) - squeeze(T_e(3, 1, j)) * z_max ./ 10], Color = 'r'); hold on
end

surface([simout.x(:, 1)';simout.x(:, 1)'], [simout.x(:, 3)';simout.x(:, 3)'], zeros(2, numel(simout.time)), [velocity';velocity'], 'FaceColor', 'no', 'EdgeColor', 'interp', 'LineWidth', 2); hold on
c = colorbar;
c.Label.String = "Speed [m/s]";

hold off
legend(["b_x", "b_y", "b_z", "Thrust"])
grid on

xlim(x_lim)
ylim([0, z_max])
axis equal
xlabel("X [m]")
ylabel("Z [m]")
title("XZ Projection")

%% YZ Projection
loops = numel(simout.time);

for j = 1:loops
    quiver([simout.x(j, 2)], [simout.x(j, 3)], [b_x(2, 1, j)] .* z_max ./ 10, [b_x(3, 1, j)] .* z_max ./ 10, Color='b'); hold on
    quiver([simout.x(j, 2)], [simout.x(j, 3)], [b_y(2, 1, j)] .* z_max ./ 10, [b_y(3, 1, j)] .* z_max ./ 10, Color='g'); hold on
    quiver([simout.x(j, 2)], [simout.x(j, 3)], [b_z(2, 1, j)] .* z_max ./ 10, [b_z(3, 1, j)] .* z_max ./ 10, Color='cyan'); hold on
    plot([simout.x(j, 2), simout.x(j, 2) - squeeze(T_e(2, 1, j)) * z_max ./ 10], [simout.x(j, 3), simout.x(j, 3) - squeeze(T_e(3, 1, j)) * z_max ./ 10], Color = 'r'); hold on
end

surface([simout.x(:, 2)';simout.x(:, 2)'], [simout.x(:, 3)';simout.x(:, 3)'], zeros(2, numel(simout.time)), [velocity';velocity'], 'FaceColor', 'no', 'EdgeColor', 'interp', 'LineWidth', 2); hold on
c = colorbar;
c.Label.String = "Speed [m/s]";

hold off
legend(["b_x", "b_y", "b_z", "Thrust"])

xlim(y_lim)
ylim([0, z_max])
axis equal
xlabel("Y [m]")
ylabel("Z [m]")
title("YZ Projection")