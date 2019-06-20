function dydt = odelc2(~,x)

%global v u;
alpha = x(3);
dydt =  [ x(4)*cos(alpha) x(4)*sin(alpha)  x(5) 0 0]';
end