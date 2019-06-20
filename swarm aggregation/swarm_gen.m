%% Swarm Control
% This script simulates the proposed control law with a swarm of robots
% with unicycle dynamics, homing according to the developed switched control law
% on linear and angular velocities, hopefully without colliding!
% Written by Dhruv Ilesh Shah (dhruv.shah@iitb.ac.in)

clear all; close all; warning('off');
addpath('misc/');
%% init
global v u;
rng(8);

% system paramters
vp = 2; % initial linear velocity, also peak
beta = 0.6; % beta, as used in ss1/2
k = 8.6; % ss2 repulsion parameter
num_robots = 50; % number of agents in the swarm
R_th = 5; % radius of homing circle
lambda = 2.1; % deceleration parameter in ss2 (constant update)
gamma = 0.2; % acceleration paramter in ss1 (convex update)

% For obstacle avoidance
R_coll = 2.5; % Factor accounting for sensitivity of detecting collision
R_coll_dead = 0.5; % Ultimate deadlock range, for which action needs to be taken
coll_cone = 85;

% iteration parameters
tsamp = 0.05;
max_iter = 400;
tspan = [0, tsamp];

% control variables
u = zeros(num_robots, 1);
v = ones(num_robots, 1) * vp;

% Randomly initialize initial state vector
R_start = 10;
Rad = rand(1, num_robots) * 20;
theta = rand(1, num_robots) * 2*pi;
alpha = (rand(1, num_robots) - 0.5) * 360;
[x, y] = pol2cart(theta, Rad + R_start);

% initial state vector
S0 = [x; y; alpha]';

% configuration vector
St = zeros(num_robots, 3);
St(:, 3) = pi/2;

% variables for storing temporary data
Rdat = zeros(num_robots, 3); % curr, prev, pprev
Rdat(:, 1) = sum(S0(:,1:2).^2, 2).^0.5;

% logging variables
logs = zeros(num_robots, 12, max_iter);
% r, inp(u), config_varsX3, delta_ij, d_ij, dotR, dotRtheta, bearing, v, ss

% Setting initial values
for i = 1:num_robots
   logs(i, :, 1) = [Rdat(i, 1), u(i), S0(i, :), 0, 0, 0, 0, 0, 0, 0]; 
end

for i = 2:max_iter
    i
    if (mod(i, 100)==0) disp(strcat('Iter: ', num2str(i))); end;
    for j = 1:num_robots
        % state computations here
        if logs(j, 1, i-1) < R_th
            [t, x1] = ode45(@odelc2, tspan, [S0(j, :), 0, 0]);
        else
            [t, x1] = ode45(@odelc2, tspan, [S0(j, :), v(j), u(j)]);
        end
        
        alpha1 = x1(end, 3);
        theta1 = atan2(x1(end, 2), x1(end, 1));

        Rdat(j, 3) = Rdat(j, 2);
        Rdat(j, 2) = Rdat(j, 1);
        Rdat(j, 1) = sqrt(x1(end, 1)^2 + x1(end, 2)^2);

        dot_R1 = v(j) * cos(alpha1 - theta1);
        dot_theta1_R1 = v(j) * sin(alpha1 - theta1);

        logs(j, 1, i) = Rdat(j, 1);
        logs(j, 2, i) = u(j);
        logs(j, 3:5, i) = x1(end, 1:3);
        logs(j, 8:9, i) = [dot_R1, dot_theta1_R1];
        logs(j, 11, i) = v(j);
        S0(j, :) = x1(end, 1:3);
    end
       
    for j = 1:num_robots
        dist_obs = sum((S0(:, 1:2) - repmat(S0(j, 1:2), num_robots, 1)).^2, 2).^0.5; % Euclidean distance to all obstacles
        dist_obs(j) = inf; % neglecting distance to self
        [dist_sorted, sort_ind] = sort(dist_obs);
        tmp_j = atan2(S0(:, 2) - S0(j, 2), S0(:, 1) - S0(j, 1));
        alpha1 = S0(j, 3);
        bearing_j = wrapTo180(alpha1*180/pi - tmp_j*180/pi);
        minind = -1;
        
        for jj = sort_ind'
            if dist_obs(jj) > R_coll
                continue
            end
            if abs(bearing_j(jj)) <= coll_cone
                minind = jj;
                break
            end
        end
        
        dot_R1 = logs(j, 8, i);
        dot_theta1_R1 = logs(j, 9, i);
        
        if (minind ~= -1)
            % linear velocity control, deceleration *lambda*
                subsystem_j = 2;
                v(j) = max(0, v(j) - lambda*tsamp); % constant update
                u(j) = (beta * sign(dot_theta1_R1)) + (k * sign(bearing_j(minind)));

                logs(j, 7, i) = dist_obs(minind);
                logs(j, 6, i) = tmp_j(minind);
                logs(j, 10, i) = bearing_j(minind);
        else
            subsystem_j = 1;
            % linear velocity control, exponential acceleration
            v(j) = gamma*v(j) + (1 - gamma)*vp; % convex update
            u(j) = (beta * sign(dot_theta1_R1));
        end
        
        logs(j, 12, i) = subsystem_j;
    end    
end
t = [1:max_iter] * tsamp;
save(strcat('results/logs-', num2str(num_robots), '-', num2str(max_iter), '.mat'), 'logs', 't', 'R_th', 'R_start');