x_init = [pi, 0];
u = [0];
x = x_init;

% constant
g = 9.81;
m = 1;
l = 1;
b = 1;
% linearized dynamics
A = [0 1;
     -g/l -b];
B = [0; 1/(m*l^2)];
Q = [5000 0; 0 1];
R = [1];
N = [0;0];
[k, s] = lqr(A, B, Q, R, N);

t_list = [];
x_list = [];
x = x_init;
x_desired = [pi, 0];
for time = 0:0.1:10
    t_span = [0, 0.1];
    x_error = x-x_desired;
    u = -k*x_error';
    disp(u);
    [t, x] = ode45(@(t, x) pendulum_dynamics(t, x, u), t_span , x);
    x = x(end,:);
    t_list = [t_list, time];
    x_list = [x_list x(end,1)];
end
plot(t_list, x_list, '-o');