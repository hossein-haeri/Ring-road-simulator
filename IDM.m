function [ acceleration ] = IDM( x, x_lead, v, v_lead)
% x = lead agent position
% v = lead agent velocity
% x_lead = lead agent position
% v_lead = lead agent velocity


a = 2;              % Max Acceleration (m/s^2)
b = 3.4;            % Breaking Acceleration (m/s^2)
v_max = 25;         % Max Velocity (m/s)
s_0 = 2;            % Safety Distance (m)
T = 2.5;            % Time Headway (s)
delta = 4;

delta_v = v - v_lead;
s = x - x_lead;

s_star = s_0 + v*T + (v * delta_v)/(2*sqrt(a*b));
acceleration = a * (1 - (v/v_max)^delta - (s_star/s)^2);

end

