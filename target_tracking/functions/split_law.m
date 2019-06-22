function dydt = split_law(~,x)
	%global v u;
	alpha = x(3);
	dydt =  [ cos(alpha) sin(alpha)  x(4) 0 ]';
end	