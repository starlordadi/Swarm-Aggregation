function rel_pos = get_relative_mat(S)
	n = size(S(:, 1));
	rel_pos = zeros(n, n, 2);
	for i = 1:n
		for j = 1:n
			rel_pos(j, i, :) = S(j, 1:2) - S(i, 1:2);
		end
	end
end