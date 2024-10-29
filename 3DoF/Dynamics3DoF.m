function state = Dynamics3DoF(state0, control, vehicle)
    % state0 - a vector containing [x, y, xdot, ydot, theta, thetadot]

    % thrust - the number of newtons of thrust

    % thrustAngle - the angle the thrust is being directed, in radians
    %   counterclockwise from the bottom of the craft.
    
    % facingAngle - the direction the top of the craft is facing, in
    %   radians
    
    arguments
        state0
        control
        vehicle {mustBeA(vehicle, "Vehicle")}
    end

    GRAVITY = 9.81;
    accel = control(1)/vehicle.m;

    xdot = state0(3);
    ydot = state0(4);

    xdotdot = accel*(cos(state0(5))*cos(control(2)) - ...
        sin(control(2))*sin(state0(5)));
    ydotdot = accel*(cos(state0(5))*sin(control(2)) + ...
        cos(control(2))*(sin(state0(5)))) - GRAVITY;

    thetadot = state0(6);
    thetadotdot = vehicle.L*control(1)*sin(control(2))/vehicle.I;

    state = [xdot ydot xdotdot ydotdot thetadot thetadotdot];
end