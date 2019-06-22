%% This script simulates the split control law for swarm agents
% Author : Adarsh Patnaik(adarshpatnaik13031999@gmail.com)
clear all; close all; warning('off');
addpath('functions/');

%%init

%parameters
global a = 0.05;
global w = 3;
global C1 = 2; global l1 = 0.5; global C2 = 1; global l2 = 1;
global b = 1.39;
num_robots = 100;
betaa = sqrt(5)/10;
max_iter = 500;
%initialize lambda
lambda = 0;

%% initialize swarm agents
u_ji = zeros(num_robots, 1);

%% create configuration vector
S0 = [x, y, alpha]';
St = zeros(num_robots, 3);
Rji_mod = zeros(num_robots, num_robots);
Rji_vec = zeros(num_robots, num_robots, 2);
%initialize distances matrix
Rji_mod = get_dist_mat(S0);
Rji_vec = get_relative_mat(S0);
u_ji = get_uji(Rji_vec, Rji_mod, S0)

for j = 1:max_iter
	j
	for i = 1:num_robots
		ui = sum(u_ji(:, i), 2) - u_ji(i, i)
		
		
