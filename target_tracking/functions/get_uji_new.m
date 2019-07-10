function uji_new = get_uji_new(vec, mag, S)
	global C1 l1 C2 l2 w a b lambda betaa num_robots;
	n = num_robots;
	uji_new = zeros(n, n);
	for i = 1:n
		for j = 1:n
			uji_new(j, i) = (C1*exp(-mag(j, i)/l1) - C2*exp(-mag(j, i)/l2))*sum(reshape(vec(j, i, :), [1, 2]).*[-sin(S(i, 3)) cos(S(i, 3))])/mag(j,i) ...
							+ sum(reshape(vec(j, i, :), [1, 2]).*[-sin(S(i, 3)) cos(S(i, 3))])*lambda*exp(-(abs(sum(reshape(vec(j, i, :), [1, 2]).*[-sin(S(j, 3)) cos(S(j, 3))])) - b)^2/(2 * betaa^2))/mag(j, i);
		end
	end
end
