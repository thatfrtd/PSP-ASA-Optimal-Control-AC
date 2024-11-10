function x_dot = Dynamics3DoF(x_current, u_current, vehicle)
    % state0 - a vector containing [x, y, xdot, ydot, theta, thetadot]

    % thrust - the number of newtons of thrust

    % thrustAngle - the angle the thrust is being directed, in radians
    %   counterclockwise from the bottom of the craft.
    
    % facingAngle - the direction the top of the craft is facing, in
    %   radians
    
    arguments
        x_current
        u_current
        vehicle {mustBeA(vehicle, "Vehicle")}
    end

    max_thrust = vehicle.max_thrust;
    L = vehicle.L;
    I = vehicle.I;
    m = vehicle.m;

    g = 9.81;
    % Extract state variables
    pos_x = x_current(1);
    pos_y = x_current(2);
    vel_x = x_current(3);
    vel_y = x_current(4);
    theta = x_current(5);
    omega = x_current(6);
    
    % Extract control variables
    thrust_percent = u_current(1);
    thrust_angle = u_current(2);
    
    % forces
    F_x = max_thrust * thrust_percent * cos(thrust_angle + theta);
    F_y = max_thrust * thrust_percent * sin(thrust_angle + theta);
    
    % torque
    T = - L * max_thrust * thrust_percent * sin(thrust_angle);
    
    % accelerations
    acc_x = F_x / m;
    acc_y = (F_y / m) - g;
    alpha = T / I;  %angular
    
    % Define the state  derivatives
    x_dot = [vel_x; vel_y; acc_x; acc_y; omega; alpha];
end