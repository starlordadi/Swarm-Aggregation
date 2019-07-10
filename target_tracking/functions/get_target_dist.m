function R = get_target_dist(targets, S)
	global num_robots;
	R = zeros(num_robots, 2);
	R(:, 1) = sum((S(:, 1:2) - repmat(targets(1, 1:2), num_robots, 1)).^2, 2).^0.5;
	R(:, 2) = sum((S(:, 1:2) - repmat(targets(2, 1:2), num_robots, 1)).^2, 2).^0.5;
end