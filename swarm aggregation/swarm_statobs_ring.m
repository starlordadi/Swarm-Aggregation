%% Swarm Control
% This script simulates the proposed control law with a swarm of robots
% with unicycle dynamics, homing according to the developed switched control law
% on linear and angular velocities in the presence of static obstacles,
% hopefully without colliding!
% Written by Dhruv Ilesh Shah (dhruv.shah@iitb.ac.in)

clear all; close all; warning('off');
addpath('misc/');
%% init
global v u;
rng(8);

% system paramters
vp = 2; % initial linear velocity, also peak
beta = 0.6; % beta, as used in ss1/2
k = 9; % ss2 repulsion parameter
R_th = 5; % radius of homing circle
lambda = 2.1; % deceleration parameter in ss2 (constant update)
gamma = 0.2; % acceleration paramter in ss1 (convex update)

% For obstacle avoidance
R_coll = 2.5; % Factor accounting for sensitivity of detecting collision
R_coll_dead = 0.5; % Ultimate deadlock range, for which action needs to be taken
coll_cone = 85;

% iteration parameters
tsamp = 0.1;
max_iter = 1200;
tspan = [0, tsamp];

% stat_obs = 50; % static obstacle size
% figure;
% viscircles([0, 0], R_th); axis(1.25 * [-R_start, R_start, -R_start, R_start]);
% x_obs = []; y_obs = [];
% % Defining obstacles
% get_obs = 'y';
% while (strcmp(get_obs, 'y'))
%     [~, x_tmp, y_tmp] = freehanddraw;
%     x_obs = vertcat(x_obs, x_tmp);
%     y_obs = vertcat(y_obs, y_tmp);
%     get_obs = input('Add more obstacles? [y/n]: ', 's');
% end
% disp(strcat(num2str(numel(x_obs)), ' points found.'));
% downsampling = input('Enter Downsampling Ratio: ');
% x_obs = x_obs(1:downsampling:end);
% y_obs = y_obs(1:downsampling:end);
load('stat_obs_path_3.mat', 'x_obs', 'y_obs');

stat_obs = numel(x_obs);
num_agents = 120; % number of agents in the swarm
num_robots = num_agents + stat_obs;

% control variables
u = zeros(num_robots, 1);
v = ones(num_robots, 1) * vp;
v(num_robots-stat_obs:num_robots) = 0;

% Randomly initialize initial state vector
R_start = 70;
alpha = (rand(1, num_robots) - 0.5) * 360;
x_rand = rand(1, num_robots) * 15;
x = R_start + x_rand;
y = rand(1, num_robots) * 90 - (45);

% % line obs
% x(num_robots-stat_obs:num_robots) = -0.5 * R_start;
% y(num_robots-stat_obs:num_robots) = linspace(0.25*R_start, -0.25*R_start, numel(num_robots-stat_obs:num_robots));

% % U - obs
% unit_1 = 20; unit_2 = 20;
% x(num_robots-stat_obs:num_robots-stat_obs+unit_1) = linspace(-0.6*R_start, -0.55*R_start, numel(num_robots-stat_obs:num_robots-stat_obs+unit_1));
% y(num_robots-stat_obs:num_robots-stat_obs+unit_1) = -0.125 * R_start;
% x(num_robots-stat_obs+unit_1+1:num_robots-stat_obs+unit_1+unit_2) = -0.55*R_start;
% y(num_robots-stat_obs+unit_1+1:num_robots-stat_obs+unit_1+unit_2) = linspace(-0.125*R_start, 0.125*R_start, numel(num_robots-stat_obs+unit_1+1:num_robots-stat_obs+unit_1+unit_2));
% x(num_robots-stat_obs+unit_1+unit_2+1:end) = linspace(-0.6*R_start, -0.55*R_start, numel(num_robots-stat_obs+unit_1+unit_2+1:num_robots));
% y(num_robots-stat_obs+unit_1+unit_2+1:end) = 0.125 * R_start;

% % General obstacle
x(num_robots-stat_obs+1:end) = x_obs;
y(num_robots-stat_obs+1:end) = y_obs;

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
    if (mod(i, 100)==0)
        disp(strcat('Iter: ', num2str(i)));
    end;
    for j = 1:num_robots-stat_obs
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
       
    for j = 1:num_robots-stat_obs
        dist_obs = sum((S0(:, 1:2) - repmat(S0(j, 1:2), num_robots, 1)).^2, 2).^0.5; % Euclidean distance to all obstacles
        dist_obs(j) = inf; % neglecting distance to self
        [minsep, minind] = min(dist_obs);
        logs(j, 7, i) = minsep;
        
        logs(j, 6, i) = atan2(S0(minind, 2) - S0(j, 2), S0(minind, 1) - S0(j, 1)); % Four quadrant inverse, hence fine.
        % caveat: we need closest neighbour INSIDE the cone
        
        alpha1 = S0(j, 3);
        bearing1 = wrapTo180(alpha1*180/pi - logs(j, 6, i)*180/pi);
        dist_thresh = R_coll * 0.5;
        logs(j, 10, i) = bearing1;
        dot_R1 = logs(j, 8, i);
        dot_theta1_R1 = logs(j, 9, i);
        
        if (minsep <= R_coll)
            % linear velocity control, deceleration *lambda*
            if (abs(bearing1) <= coll_cone)
                subsystem_j = 2;
                v(j) = max(0, v(j) - lambda*tsamp); % constant update
                u(j) = (beta * sign(dot_theta1_R1)) + (k * sign(bearing1));
            else
                subsystem_j = 1;
                % linear velocity control, exponential acceleration
                v(j) = gamma*v(j) + (1 - gamma)*vp; % convex update
                u(j) = (beta * sign(dot_theta1_R1));
            end
        else
            subsystem_j = 1;
            % linear velocity control, exponential acceleration
            v(j) = gamma*v(j) + (1 - gamma)*vp; % convex update
            u(j) = (beta * sign(dot_theta1_R1));
        end
        logs(j, 12, i) = subsystem_j;
    end    
end

close all; figure;
% Visualising end position markers
plot(logs(1:num_robots-stat_obs, 3, max_iter), logs(1:num_robots-stat_obs, 4, max_iter), 'o', 'MarkerSize', 3); hold on;
plot(logs(num_robots-stat_obs+1:end, 3, max_iter), logs(num_robots-stat_obs+1:end, 4, max_iter), 'k.', 'MarkerSize', 14);
axis(1.25 * [-R_start, R_start, -R_start, R_start]); viscircles([0, 0], R_th);