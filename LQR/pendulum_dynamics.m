function dxdt = pendulum_dynamics(t,x,u)
    g = 9.81;
    m = 1;
    l = 1;
    b = 1;
    dxdt = zeros(2,1);
    dxdt(1) = x(2);
    dxdt(2) = -(g/l)*sin(x(1)) + u/(m*l^2) - b*x(2)/(m*l^2);
end