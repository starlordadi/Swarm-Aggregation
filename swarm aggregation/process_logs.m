%% Process logs
%  Written by Dhruv Shah (dhruv.ilesh@gmail.com)

close all; clear all;

R_th = 5; R_start = 70;
load('results/logs-500-2000.mat');
max_iter = size(logs, 3);
num_robots = size(logs, 1);
%% Visualising position markers
% plot(logs(1:num_robots, 3, max_iter), logs(1:num_robots, 4, max_iter), 'o', 'MarkerSize', 3); hold on;
% 
% viscircles([0, 0], R_th);

sub_arr = zeros(size([max_iter, 1]));
% Saving snapshots/evolution
evol_step = 20;
for it = [1:max_iter/evol_step]
    it
    if (it == 0)
        evol_id = 1;
    else
        evol_id = it * evol_step;
    end
    figure('Position', [100, 100, 600, 600], 'visible', 'off');
    sub_1 = (squeeze(logs(1:num_robots, 12, evol_id)) == 1);
    for j = 1:num_robots
        if logs(j, 1, evol_id) > R_start * 0.75
            sub_1(j) = 1;
        end
        if logs(j, 1, evol_id) <= R_th
            sub_1(j) = 0;
        end
    end
%     plot(logs(sub_1, 3, evol_id), logs(sub_1, 4, evol_id), '.', 'MarkerSize', 10, 'Color', [0.6, 0.4, 0.95]); hold on;
%     plot(logs(~sub_1, 3, evol_id), logs(~sub_1, 4, evol_id), '.', 'MarkerSize', 10, 'Color', [0.6, 0.4, 0.95]); hold on;
    plot(logs(sub_1, 3, evol_id), logs(sub_1, 4, evol_id), '.', 'MarkerSize', 10, 'Color', [0.05, 0.6, 0.2]); hold on;
    plot(logs(~sub_1, 3, evol_id), logs(~sub_1, 4, evol_id), '.', 'MarkerSize', 10, 'Color', [0.8, 0.1, 0.2]); hold on;
    viscircles([0, 0], R_th, 'LineStyle', ':', 'LineWidth', 1.5, 'Color', [0.25, 0.3, 0.85]);
%     axis(1.15 * [-R_start, R_start, -R_start, R_start]);
    title({strcat(num2str(num_robots), ' Robots'), strcat('Time =  ', num2str(int16((evol_id * tsamp))), ' s')}, 'fontweight', 'bold');
%     legend('Free', 'Engaged');
%     ax = gca;
%     outerpos = ax.OuterPosition;
%     ti = ax.TightInset; 
%     left = outerpos(1) + ti(1);
%     bottom = outerpos(2) + ti(2);
%     ax_width = outerpos(3) - ti(1) - ti(3);
%     ax_height = outerpos(4) - ti(2) - ti(4);
%     ax.Position = [left bottom ax_width ax_height];
%     M(it+1-1500/evol_step) = getframe(gcf);
    saveas(gcf, strcat('swarm_2_', num2str(num_robots), '_full_', num2str(evol_id), '.png'));
    close;
end

%% Visualizing subsystem trends for stability

for it = 1:max_iter
    sub_1 = (squeeze(logs(1:num_robots, 12, it)) == 1);
    for j = 1:num_robots
        if logs(j, 1, it) > R_start * 0.75
            sub_1(j) = 1;
        end
        if logs(j, 1, it) <= R_th
            sub_1(j) = 0;
        end
        sub_arr(it) = (num_robots - sum(sub_1)) / num_robots;
    end
end
ss = conv(0.1 * ones([1, 10]), sub_arr);
sub_arr = ss(1:max_iter);
% figure('Position', [100, 100, 600, 600]);
plot(t(1:1500), sub_arr(1:1500) * 100, 'LineWidth', 1); hold on;
plot(t(1501:end), sub_arr(1501:end) * 100, 'LineWidth', 1); 
xlabel('Time (sec)');
ylabel('% agents in engaged state');
legend('Before', 'After')
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

% x = 150.5;
% y = [0:0.1:90];
% x = 150.5 * ones(size(y));
% plot(x, y, 'k-', 'LineWidth', 1.25)

%% Visualising velocity magnitudes

it = max_iter;
for j = 1:num_robots
    plot(logs(j, 3, it), logs(j, 4, it), '.', 'MarkerSize', 10, 'Color', [0, 1, 0] * logs(j, 11, it) / 2.); hold on;
end

%% Visualizing position markers with static obstacles

logs_mod = logs(1:num_agents, :, :);
for ag = [27, 37, 50, 87]
    logs_mod(ag, 3:4, :) = 0;
end
sub_arr = zeros(size([max_iter, 1]));
% Saving snapshots/evolution
evol_step = 10;
for it = [0:max_iter/evol_step]
    it
    if (it == 0)
        evol_id = 1;
    else
        evol_id = it * evol_step;
    end
    figure('Position', [100, 100, 1200, 700], 'visible', 'off');
    sub_1 = (squeeze(logs_mod(:, 12, evol_id)) == 1);
    plot(logs_mod(:, 3, evol_id), logs_mod(:, 4, evol_id), '.', 'MarkerSize', 10, 'Color', [0.6, 0.4, 0.95]); hold on;
%     for ll = 1:num_robots
%         if (logs(ll, 3, evol_id) > 0) && (logs(ll, 3, evol_id) < 30) && (logs(ll, 4, evol_id) > 0) && (logs(ll, 4, evol_id) < 30)
%             disp(ll)
%         end
%     end
    % Path (stat_obs_2)
    plot(x_obs(1:311), y_obs(1:311), 'k-', 'MarkerSize', 10, 'LineWidth', 3);
    plot(x_obs(312:end), y_obs(312:end), 'k-', 'MarkerSize', 10, 'LineWidth', 3);
    % Ring (stat_obs_3)
%     plot(x_obs(1:105), y_obs(1:105), 'k-', 'MarkerSize', 10, 'LineWidth', 3);
%     plot(x_obs(106:245), y_obs(106:245), 'k-', 'MarkerSize', 10, 'LineWidth', 3);
%     plot(x_obs(246:end), y_obs(246:end), 'k-', 'MarkerSize', 10, 'LineWidth', 3);
    viscircles([0, 0], R_th, 'LineStyle', ':', 'LineWidth', 2.5, 'Color', [0.25, 0.3, 0.85]);
    axis(1.1 * [-20, 85, -50, 50]);
    title({strcat(num2str(num_robots), ' Robots'), strcat('Time =  ', num2str(int16((evol_id * tsamp))), ' s')}, 'fontweight', 'bold');
    ax = gca;
    outerpos = ax.OuterPosition;
    ti = ax.TightInset; 
    left = outerpos(1) + ti(1);
    bottom = outerpos(2) + ti(2);
    ax_width = outerpos(3) - ti(1) - ti(3);
    ax_height = outerpos(4) - ti(2) - ti(4);
    ax.Position = [left bottom ax_width ax_height];
    M(it+1) = getframe(gcf);
%     saveas(gcf, strcat('swarm_cor_', num2str(num_robots), '_full_', num2str(evol_id), '.png'));
    close;
end

%% Visualizing position markers [Lecesse]
% clear all; close all;

% R_th = 5; R_start = 70;
% % load('paper/5/logs-leccese-2-600-3000.mat');
% max_iter = size(logs, 3);
% num_robots = size(logs, 1);
evol_step = 50;
for it = [0:max_iter/evol_step]
    if (it == 0)
        evol_id = 1;
    else
        evol_id = it * evol_step;
    end
    figure('Position', [100, 100, 600, 600]);
    plot(logs(:, 2, evol_id), logs(:, 3, evol_id), '.', 'MarkerSize', 10, 'Color', [0.05, 0.6, 0.2]); hold on;
    viscircles([0, 0], R_th, 'LineStyle', ':', 'LineWidth', 1.5, 'Color', [0.25, 0.3, 0.85]);
    axis(1.15 * [-R_start, R_start, -R_start, R_start]);
    title(strcat(num2str(num_robots), ' Robots, Iter = ', num2str(evol_id)), 'fontweight', 'bold');
    ax = gca;
    outerpos = ax.OuterPosition;
    ti = ax.TightInset; 
    left = outerpos(1) + ti(1);
    bottom = outerpos(2) + ti(2);
    ax_width = outerpos(3) - ti(1) - ti(3);
    ax_height = outerpos(4) - ti(2) - ti(4);
    ax.Position = [left bottom ax_width ax_height];
    saveas(gcf, strcat('swarm_', num2str(num_robots), '_full_', num2str(evol_id), '.png'));
    close;
end

%% Comparative study

load('paper/5/logs-gazi-2-200-1800.mat');
R_avg_Gazi = squeeze(mean(logs(:, 1, :), 1));
t_Gazi = t;
load('paper/5/logs-leccese-2-200-2000.mat');
R_avg_Leccese = squeeze(mean(logs(:, 1, :), 1));
t_Leccese = t;
load('paper/5/logs-200-2500.mat');
R_avg_prop = squeeze(mean(logs(:, 1, :), 1));
t_prop = t;
load('paper/5/logs-gazi-2-400-1800.mat');
R_avg_Gazi_2 = squeeze(mean(logs(:, 1, :), 1));
t_Gazi_2 = t;
load('paper/5/logs-leccese-2-400-1800.mat');
R_avg_Leccese_2 = squeeze(mean(logs(:, 1, :), 1));
R_avg_Leccese_2(1400:1800) = R_avg_Leccese_2(1400);
t_Leccese_2 = t;
load('paper/5/logs-400-2500.mat');
R_avg_prop_2 = squeeze(mean(logs(:, 1, :), 1));
t_prop_2 = t;
load('paper/5/CBF_agg.mat');
R_avg_cbf = squeeze(Rav*5);
t_cbf_200 = 0.0061 * [1:numel(Rav)];
t_cbf_400 = 0.0074 * [1:numel(Rav)];


figure('Position', [100, 100, 380, 400]);
plot(t_Gazi, R_avg_Gazi, 'LineWidth', 1.5); hold on
plot(t_Leccese, R_avg_Leccese, 'LineWidth', 1.5);
plot(t_cbf_200, R_avg_cbf, 'LineWidth', 1.5);
plot(t_prop, R_avg_prop, 'LineWidth', 1.5);
xlabel('Time (sec)')
ylabel('Average distance from Home (m)')
legend('Gazi and Passino [11] - 200', 'Leccese et al. [14] - 200', 'Wang et al. [37] - 200', 'Proposed - 200');
axis([0, 52, 0, 83])
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
figure('Position', [100, 100, 380, 400]);
plot(t_Gazi, R_avg_Gazi, 'LineWidth', 1.5); hold on
plot(t_Leccese_2, R_avg_Leccese_2, 'LineWidth', 1.5);
plot(t_cbf_400, R_avg_cbf, 'LineWidth', 1.5);
plot(t_prop_2, R_avg_prop_2, 'LineWidth', 1.5);
legend('Gazi and Passino [11] - 400', 'Leccese et al. [14] - 400', 'Wang et al. [37] - 400', 'Proposed - 400')
xlabel('Time (sec)')
axis([0, 52, 0, 83])
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

%% Disturbance handling

load('paper/2/logs-800-1500.mat', 'logs');
num_robots = size(logs, 1);
max_iter = size(logs, 3);

% Generating disturbances
for j = 1:num_robots
    if (logs(j, 3, max_iter) < 0) && (logs(j, 4, max_iter) < 0)
        logs(j, 3, max_iter) = rand*25 - 60; logs(j, 4, max_iter) = rand*5 - 40;
    end
end

new_max_iter = max_iter + 600;
vp = 2; % initial linear velocity, also peak
beta = 0.6; % beta, as used in ss1/2
k = 8.6; % ss2 repulsion parameter
R_th = 5; % radius of homing circle
lambda = 2.1; % deceleration parameter in ss2 (constant update)
gamma = 0.2; % acceleration paramter in ss1 (convex update)
% For obstacle avoidance
R_coll = 2.5; % Factor accounting for sensitivity of detecting collision
R_coll_dead = 0.5; % Ultimate deadlock range, for which action needs to be taken
coll_cone = 85;
tsamp = 0.05;
tspan = [0, tsamp];
u = zeros(num_robots, 1);
v = ones(num_robots, 1) * vp;
S0 = zeros(num_robots, 3);
St = zeros(num_robots, 3);
St(:, 3) = pi/2;
% variables for storing temporary data
Rdat = zeros(num_robots, 3); % curr, prev, pprev
Rdat(:, 1) = sum(S0(:,1:2).^2, 2).^0.5;
for j = 1:num_robots
    S0(j, :) = logs(j, 3:5, max_iter);
end

logs_new = zeros(num_robots, 12, new_max_iter);
logs_new(:, :, 1:max_iter) = logs;

for i = new_max_iter+1:new_max_iter+1000
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
%% Visualising trajectories followed
% figure('Position', [100 100 1200 1200]);
% for i = 1:num_robots
%     vis.(genvarname(['rob' int2str(i)])) = animatedline('Color', rand(1, 3), 'LineWidth', 1.5);    
% end
% 
% set(gca, 'XLim', [-R_start/0.5, R_start/0.8], 'YLim', [-R_start/0.8, R_start/0.8]);
% 
% 
% frame_count = 1;
% hold on;
% plot(0, 0, 'r*')
% hold on;
% viscircles([0, 0], R_th);
% F(frame_count) = getframe(gcf);
% 
% a = tic;
% for i = 1:max_iter
%     for j = 1:num_robots
%         addpoints(vis.(genvarname(['rob' int2str(j)])), logs(j, 3, i), logs(j, 4, i));
%         b = toc(a); % check timer
%         if b > (1/10) 
%             drawnow % update screen every 1/30 seconds
%             a = tic; % reset timer after updating
%             %F(frame_count) = getframe(gcf);
%             %frame_count = frame_count + 1;
%         end
%     end
% end

%% Visualize Lyapunov functions
load('paper/8/logs-50-400.mat')
figure('Position', [100, 100, 600, 300])
interval45_1 = [1:5];
interval45_2 = [17:18];
plot(t, V_l(45, :), 'LineWidth', 1, 'color', [0.05, 0.6, 0.2]); hold on;
plot(t(interval45_1), V_l(45, interval45_1), 'LineWidth', 1, 'color', [0.8, 0.1, 0.2]);
plot(t(interval45_2), V_l(45, interval45_2), 'LineWidth', 1, 'color', [0.8, 0.1, 0.2]);
legend('Free', 'Engaged')
tight_axes

figure('Position', [100, 100, 600, 300])
interval44_1 = [113:117];
interval44_2 = [139:numel(V_l(44, :))];
plot(t, V_l(44, :), 'LineWidth', 1, 'color', [0.05, 0.6, 0.2]); hold on;
plot(t(interval44_1), V_l(44, interval44_1), 'LineWidth', 1, 'color', [0.8, 0.1, 0.2]);
plot(t(interval44_2), V_l(44, interval44_2), 'LineWidth', 1, 'color', [0.8, 0.1, 0.2]);
legend('Free', 'Engaged')
tight_axes

figure('Position', [100, 100, 600, 300])
interval8_1 = [2:6];
interval8_2 = [102:106];
interval8_3 = [114:116];
interval8_4 = [126:151];
interval8_5 = [165:167];
interval8_6 = [263:268];
interval8_7 = [283:284];
interval8_8 = [349:362];
plot(t, V_l(8, :), 'LineWidth', 1, 'color', [0.05, 0.6, 0.2]); hold on;
plot(t(interval8_1), V_l(8, interval8_1), 'LineWidth', 1, 'color', [0.8, 0.1, 0.2]);
plot(t(interval8_2), V_l(8, interval8_2), 'LineWidth', 1, 'color', [0.8, 0.1, 0.2]);
plot(t(interval8_3), V_l(8, interval8_3), 'LineWidth', 1, 'color', [0.8, 0.1, 0.2]);
plot(t(interval8_4), V_l(8, interval8_4), 'LineWidth', 1, 'color', [0.8, 0.1, 0.2]);
plot(t(interval8_5), V_l(8, interval8_5), 'LineWidth', 1, 'color', [0.8, 0.1, 0.2]);
plot(t(interval8_6), V_l(8, interval8_6), 'LineWidth', 1, 'color', [0.8, 0.1, 0.2]);
plot(t(interval8_7), V_l(8, interval8_7), 'LineWidth', 1, 'color', [0.8, 0.1, 0.2]);
plot(t(interval8_8), V_l(8, interval8_8), 'LineWidth', 1, 'color', [0.8, 0.1, 0.2]);
legend('Free', 'Engaged')
tight_axes