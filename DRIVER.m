function [ acceleration ] = DRIVER(type, x, x_leader, v, v_leader)

if type == 'IDM'
        % x = lead agent position
        % v = lead agent velocity
        % x_lead = lead agent position
        % v_lead = lead agent velocity

        a = 1;             % Max Acceleration (m/s^2)
        b = 3.4;            % Breaking Acceleration (m/s^2)
        v_max = 15;         % Max Velocity (m/s)
        s_0 = 2;            % Safety Distance (m)
        T = 2.5;              % Time Headway (s)
        delta = 4;
        vehicle_length = 5;

        delta_v = v - v_leader;
        s = x - x_leader - vehicle_length;

        s_star = s_0 + v*T + (v * delta_v)/(2*sqrt(a*b));
        acceleration = a * (1 - (v/v_max)^delta - (s_star/s)^2);
end


if type == 'OVM'
        a = 2;
        V = 16.8*(tanh((0.0860)*(x_leader-x - 25))+0.913);
        acceleration = a*(V - v);
end

end