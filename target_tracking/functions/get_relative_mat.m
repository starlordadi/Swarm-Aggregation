function rel_pos = get_relative_mat(S)
	global num_robots;
	n = num_robots;
	rel_pos = zeros(n, n, 2);
	for i = 1:n
		for j = 1:n
			rel_pos(j, i, :) = S(i, 1:2) - S(j, 1:2);
		end
	end
end

