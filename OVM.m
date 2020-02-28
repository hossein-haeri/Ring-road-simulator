function [ acceleration ] = OVM(x, x_leader, v, ~)
a = 3;
V = 16.8*(tanh(0.0860)*(x_leader-x - 25)+0.913);
acceleration = a*(V - v);
end