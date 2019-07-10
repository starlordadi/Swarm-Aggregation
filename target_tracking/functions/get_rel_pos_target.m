function rel_pos_target = get_rel_pos_target(S, targets)
	global num_robots;
	rel_pos_target = zeros(2, num_robots, 2);
	rel_pos_target(1, :, :) = S(:, 1:2) - repmat(targets(1, :), num_robots, 1);
	rel_pos_target(2, :, :) = S(:, 1:2) - repmat(targets(2, :), num_robots, 1);
end