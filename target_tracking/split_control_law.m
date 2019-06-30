%% This script simulates the split control law for swarm agents
% Author : Adarsh Patnaik(adarshpatnaik13031999@gmail.com)
clear all; close all; warning('off');
addpath('functions/');

%%init

%parameters
global lambda a b w C1 l1 C2 l2 betaa num_robots;
b = 1.39;
a = 0.05;
w = 3;
C1 = 2; l1 = 0.5; C2 = 1; l2 = 1;
num_robots = 100;
betaa = sqrt(5)/10;

max_iter = 500;
tsamp = 0.01;
tspan = [0, tsamp];
%initialize lambda
lambda = 0;

%% initialize swarm agents
u_ji = zeros(num_robots, 1);
x = (rand(1, num_robots)*2 - 1);
y = (rand(1, num_robots)*2 - 1);
alpha = (rand(1, num_robots)*2 - 1) * (pi/4);
%% create configuration vector
S0 = [x; y; alpha]';
St = zeros(num_robots, 3);
Rji_mod = zeros(num_robots, num_robots);
Rji_vec = zeros(num_robots, num_robots, 2);
%initialize distances matrix

for j = 1:max_iter
	j
	Rji_mod = get_dist_mat(S0);
	Rji_vec = get_relative_mat(S0);
	u_ji = get_uji(Rji_vec, Rji_mod, S0);

	for i = 1:num_robots
		ui = sum(u_ji(:, i)) - u_ji(i, i);
		[t, x1] = ode45(@split_law, tspan, [S0(i, :), ui]);
		S0(i, :) = x1(end, 1:3);
	end
end
		
		
