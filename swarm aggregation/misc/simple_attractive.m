function dydt = simple_attractive(~, y)
% A simple attractive function
% \dot{y} = -\frac{y}{\norm2{y}^2}

    dydt = - y * (1 - exp(-sum(y.^2)));
end