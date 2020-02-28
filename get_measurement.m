function z = get_measurement(position, std_friction, std_elevation)

[mu_friction, mu_elevation] = road_truth(position);

z(1) = normrnd(mu_friction, std_friction);
z(2) = normrnd(mu_elevation, std_elevation);
end


function [friction, elevation] = road_truth(position)
% road friction as a function of location
friction = sin(position) + 2; 
% road elevation as a function of location
elevation = 0.01 * cos(position) + 330;
end