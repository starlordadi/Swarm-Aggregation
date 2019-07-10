function u_ik = get_uik(vec, mag, S)
	global gamm_a r0 num_robots;
	u_ik = zeros(num_robots, 1);
	for i = 1:num_robots
		if(mag(i, 1) <= mag(i, 2))
			u_ik(i, 1) = gamm_a*(1 - (r0/mag(i, 1))^2)*(sum(reshape(vec(1, i, :), [1, 2]).*[-sin(S(i, 3)) cos(S(i, 3))]))/mag(i, 1);
		else
			u_ik(i, 1) = gamm_a*(1 - (r0/mag(i, 2))^2)*(sum(reshape(vec(2, i, :), [1, 2]).*[-sin(S(i, 3)) cos(S(i, 3))]))/mag(i, 2);
		end	
	end
end