%% This script simulates the split control law for swarm agents
% Author : Adarsh Patnaik(adarshpatnaik13031999@gmail.com)
clear all; close all; warning('off');
addpath('functions/');

%%init

%parameters
global lambda a b w C1 l1 C2 l2 betaa num_robots gamm_a r0;
b = 1.39;
a = 0.05;
w = 3;
C1 = 2; l1 = 0.5; C2 = 1; l2 = 1;
num_robots = 100;
betaa = sqrt(5)/10;
gamm_a = 0.1;
r0 = 0.1;

max_iter = 1000;
tsamp = 0.05;
tspan = [0, tsamp];
%initialize lambda
lambda = 0;

logs = zeros(num_robots, 4, max_iter);

%% initialize swarm agents
u_ji = zeros(num_robots, num_robots);
x = (rand(1, num_robots)*2 - 1);
y = (rand(1, num_robots)*2 - 1);
targets = zeros(2, 2);
targets(1, 1) = 50;
targets(1, 2) = 6;
targets(2, 1) = 50;
targets(2, 2) = -6;
% alpha = (rand(1, num_robots)*2 - 1) * (pi/4);
alpha = zeros(1, num_robots);
%% create configuration vector
S0 = [x; y; alpha]';
St = zeros(num_robots, 3);
%initialize distances matrix
Rji_mod = zeros(num_robots, num_robots);
Rji_vec = zeros(num_robots, num_robots, 2);
R_ik_mod = zeros(num_robots, 2);
R_ik_vec = zeros(2, num_robots, 2);
%figure('Position', [100, 100, 600, 600], 'visible', 'off');
for j = 1:max_iter
	j
	Rji_mod = get_dist_mat(S0);
	Rji_vec = get_relative_mat(S0);
	u_ji = get_uji_new(Rji_vec, Rji_mod, S0);
	R_ik_mod = get_target_dist(targets, S0);
	R_ik_vec = get_rel_pos_target(S0, targets);
	u_ik = get_uik(R_ik_vec, R_ik_mod, S0);
	for i = 1:num_robots
		ui = sum(u_ji(1:i-1, i)) + sum(u_ji(i+1:end, i)) + u_ik(i, 1);
		[t, x1] = ode45(@split_law, tspan, [S0(i, :), ui]);
		S0(i, :) = x1(end, 1:3);
		logs(i, 1:3, j) = x1(end, 1:3);
        logs(i, 4, j) = lambda;
    end
	
	S0(:, 3) = wrapTo180(S0(:, 3));

	if (j > 200 & j < 600)
		'splitting'            
		%plot(logs(:, 2, j), logs(:, 1, j), '.', 'MarkerSize', 5, 'Color', [0, 0, 1]); hold on;
		% plot(logs(:, 2, j), j*tsamp, '.', 'MarkerSize', 5, 'Color', [0, 0, 1]); hold on;
  %       xlim([-inf, inf]);
		% ylim([-inf, inf]);

		lambda = 0.29;
	else
		%plot(logs(:, 2, j), logs(:, 1, j), '.', 'MarkerSize', 5, 'Color', [0, 1, 0]); hold on;
  %       plot(logs(:, 2, j), j*tsamp, '.', 'MarkerSize', 5, 'Color', [0, 1, 0]); hold on;
		% xlim([-inf, inf]);
		% ylim([-inf, inf]);

		lambda = 0;
	end

	if (j > 400)
		b = 3.39;
	end
	% if(mod(j, 50) == 0)
	% 	figure('Position', [100, 100, 600, 600], 'visible', 'off');
	% 	plot(logs(:, 2, j), logs(:, 1, j), '.', 'MarkerSize', 5, 'Color', [0, 1, 0]); hold on;
	% 	title(strcat('at time = ', num2str(j/50)), 'fontweight', 'bold');
	% 	saveas(gcf, strcat('position_at_', num2str(j/50), '_sec.png'));
	% 	close;
 %    end

end

figure('Position', [100,100,600,600], 'visible', 'off');
plot(targets(:, 1), targets(:, 2), '.', 'MarkerSize', 15, "Color", [1, 0, 0]); hold on;
for k = 1:num_robots
	x_plot(:) = logs(k, 1, :);
	y_plot(:) = logs(k, 2, :);
	plot(x_plot, y_plot, '.', 'MarkerSize', 1, 'Color', [0, 0, 1]); hold on;
end
saveas(gcf, 'path_target.png');
close;

% figure('Position', [100,100,600,600], 'visible', 'off');
% for m = 1:num_robots
% 	x_plot_1(:) = logs(m, 1, 300:700);
% 	y_plot_1(:) = logs(m, 2, 300:700);
% 	plot(x_plot_1, y_plot_1, '.', 'MarkerSize', 1, 'Color', [0, 0, 1]); hold on;
% end
% saveas(gcf, 'split_2.png');
% close;

% figure('Position', [100,100,600,600], 'visible', 'off');
% for n = 1:max_iter
% 	plot(n, logs(1, 4, n), '.', 'MarkerSize', 1, 'Color', [0, 0, 1]); hold on;
% end
% saveas(gcf, 'lambda_2.png');
% close;
%saveas(gcf, 'y_vs_t.png');
%close;
		
		
