function y = road_truth(position)
% road friction as a function of location
y(1) = sin(position); 
% road elevation as a function of location
y(2) = 0.01 * cos(position);
end