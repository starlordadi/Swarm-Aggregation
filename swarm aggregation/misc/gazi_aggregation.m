function dydt = gazi_aggregation(~, y, agent_pos)
% Potential function as used in (Gazi and Passino, 2003)
% \dot{y} = -\frac{y}{\norm2{y}^2}
    a = 2; b = 30; c = 30;
    dydt = zeros(size(y));
    for i = [1:size(agent_pos, 1)]
        delY = y - agent_pos(i, :)';
        dydt = dydt - (delY * (a - b*exp(-sum(delY.^2) / c)));
    end
    dydt = dydt / size(agent_pos, 1);
    dydt = dydt - 3*(y / norm(y))*(1 - 20*exp(-norm(y)^2/55));
end