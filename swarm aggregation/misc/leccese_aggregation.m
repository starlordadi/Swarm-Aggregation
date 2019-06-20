function dydt = leccese_aggregation(~, y, agent_pos)
% Using local interactions for swarm aggregation (Leccese et al, 2013)
    
    a = 3; b = 20; k = 3; c = 55;
    diff_vec = repmat(y', [size(agent_pos, 1), 1]);
    diff_vec = diff_vec - agent_pos;
    diff_vec = sum(diff_vec.^2, 2);
    dydt = zeros(size(y));
    for i = [1:size(agent_pos, 1)]
        delY = y - agent_pos(i, :)';
        term = delY * ((b+a)*exp(-sum(delY.^2) / c) - a) / (diff_vec(i));
        dydt = dydt + term;
    end
    dydt = dydt / sum(1 ./ diff_vec.^0.5);
    dydt = dydt - k*(y / norm(y))*(1 - exp(-norm(y)^2/6000));
end