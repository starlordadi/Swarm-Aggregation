function dist_mat = get_dist_mat(S)
	global num_robots;
	n = num_robots;
	dist_mat = zeros(n, n);
	for j = 1:n
		diff_mat = sum((S(:, 1:2) - repmat(S(j, 1:2), n, 1)).^2, 2).^0.5;
		dist_mat(:, j) = diff_mat(:, 1);
	end
end

