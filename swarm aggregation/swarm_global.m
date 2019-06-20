%% Swarm Control
% This script simulates a simple attractive aggregation routine.
% Written by Dhruv Ilesh Shah (dhruv.shah@iitb.ac.in)

clear all; close all;
addpath('misc/');
%% init
rng(8)
% system parameters
num_robots = 50;
R_start = 50;
R_th = 5;
theta = rand(1, num_robots) * pi;
[x, y] = pol2cart(theta, R_start);

% iteration parameters
tsamp = 0.1;
max_iter = 1000;
tspan = [0, tsamp];

% initial state vector
S0 = [x; y]';

% configuration vector
St = zeros(num_robots, 2);

% logging variables
logs = zeros(num_robots, 3, max_iter);
% r, x, y
for i = 1:num_robots
    logs(i, :, 1) = [sum(S0(i, :).^2)^0.5, x(i), y(i)];
end

for i = 2:max_iter
    if (mod(i, 100)==0) disp(strcat('Iter: ', num2str(i))); end;
    for j = 1:num_robots
        [t, pos1] = ode45(@simple_attractive, tspan, S0(j, :));
        x1 = pos1(end, 1);
        y1 = pos1(end, 2);
        if (logs(j, 1, i-1) < R_th)
            x1 = S0(1); y1 = S0(2);
        end
        
        S0(j, :) = pos1(end, :);
        logs(j, 2:3, i) = S0(j, :);
    end
end