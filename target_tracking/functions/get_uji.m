function uji = get_uji(vec, mag, S)
	global C1, l1, C2, l2, w, a, lambda, betaa;
	n = size(S(:,1));
	uji = zeros(n, n);
	for i = 1:n
		for j = 1:n
			if mag(j, i) <= w
				uji(j, i) = - a*sum((vec(j, i, :).*[-sin(S(i, 3)) cos(S(i, 3))]), 2)*sum((vec(j, i, :).*[cos(S(i, 3)) sin(S(i, 3))]), 2)/(mag(j, i)^2) ...
							+ a * sum([cos(S(j, 3)) sin(S(j, 3))].*[-sin(S(i, 3)) cos(S(i, 3))], 2)/(mag(j, i)) ...
							+ (C1*exp(-mag(j, i)/l1) - C2*exp(-mag(j, i)/l2))*sum(vec(j, i, :).*[-sin(S(i, 3)) cos(S(i, 3))], 2) ...
							+ sum(vec(j, i, :).*[-sin(S(i, 3)) cos(S(i, 3))], 2)*lambda*exp(-(mod(sum(vec(j, i, :).*[-sin(S(j, 3)) cos(S(j, 3))], 2)) - b)^2/(2 * betaa^2))/mag(j, i);

			else
				uji(j, i) = (C1*exp(-mag(j, i)/l1) - C2*exp(-mag(j, i)/l2))*sum(vec(j, i, :).*[-sin(S(i, 3)) cos(S(i, 3))], 2) ...
							+ sum(vec(j, i, :).*[-sin(S(i, 3)) cos(S(i, 3))], 2)*lambda*exp(-(mod(sum(vec(j, i, :).*[-sin(S(j, 3)) cos(S(j, 3))], 2)) - b)^2/(2 * betaa^2))/mag(j, i);
			end	
		end
	end							