%% Swarm Control
% This script simulates a swarm aggregation routine based on local
% interaction and artifical potential functins (Leccese et al., 2013).
% Written by Dhruv Ilesh Shah (dhruv.shah@iitb.ac.in)

clear all; close all;
addpath('misc/');
%% init
rng(8)
% system parameters
vp = 2;
num_robots = 200;
R_start = 70;
R_th = 5;
R_coll = 3;
theta = rand(1, num_robots) * 2 * pi;
R_rand = rand(1, num_robots) * 20;
[x, y] = pol2cart(theta, R_start + R_rand);

% iteration parameters
tsamp = 0.03;
max_iter = 1800;
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
    i
    if (mod(i, 100)==0) disp(strcat('Iter: ', num2str(i))); end;
    for j = 1:num_robots
        if (norm(S0(j, :)) > R_th)
            logs_tmp = logs(:, 2:3, i-1);
            diff_pos = repmat(S0(j, :), [num_robots, 1]) - logs_tmp;
            logs_N = zeros(size(logs_tmp)); count_N = 1;
            for jj = 1:num_robots
                if jj == j
                    continue
                end
                if diff_pos(jj) < R_coll
                    count_N = count_N + 1;
                    logs_N(count_N, :) = logs_tmp(jj, :);
                end
            end
    %         disp(count_N);
            logs_N = logs_N(1:count_N, :); t = 0;
            del_pos = leccese_aggregation(t, S0(j, :)', logs_N);
            if (norm(del_pos) > vp)
                del_pos = del_pos * vp / norm(del_pos);
            end
    %         [t, pos1] = ode45(@(t, y) leccese_aggregation(t, y, logs_N), tspan, S0(j, :));
    %         x1 = pos1(end, 1);
    %         y1 = pos1(end, 2);

            S0(j, :) = S0(j, :) + del_pos'*tsamp;
        end
        logs(j, 2:3, i) = S0(j, :);
        logs(j, 1, i) = sqrt(sum(S0(j, :).^2));
    end
end
t = [1:max_iter] * tsamp;
save(strcat('results/logs-leccese-2-', num2str(num_robots), '-', num2str(max_iter), '.mat'), 'logs', 't', 'R_th', 'R_start');