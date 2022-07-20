% Kinematic vehicle states are [x, y, theta]
% x, y are position in the local frame. Theta is heading.
% Vehicle input is steering sigma.
% vehicle trim condition:
% vx, vy constant, velocity constant, theta constant, steering 0
lr = 1.5;
lf = 1;
L = lr+lf;

v_trim = 1;
sigma_trim = 0;
theta_trim = 0;
beta_trim = 0;

dbeta_dsigma = lr * L / (L^2*cos(sigma_trim)^2 + lr^2*sin(sigma_trim)^2);

dF1_dx = 0;
dF1_dy = 0;
dF1_dtheta = -v_trim*sin(theta_trim + beta_trim);
dF1_dsigma = -v_trim*sin(theta_trim + beta_trim) * dbeta_dsigma;

dF2_dx = 0;
dF2_dy = 0;
dF2_dtheta = v_trim*cos(theta_trim + beta_trim);
dF2_dsigma = v_trim*cos(theta_trim + beta_trim)*dbeta_dsigma;

dF3_dx = 0;
dF3_dy = 0;
dF3_dtheta = 0;
dF3_dsigma = v_trim / L * cos(beta_trim)*sec(sigma_trim)^2;

A = [dF2_dy dF2_dtheta; ...
     dF3_dy dF3_dtheta];
B = [dF2_dsigma; dF3_dsigma];

Q = [2 0; ...
     0 15];
R = [50];
N = [0; 0];

[K,S,e] = lqr(A, B, Q, R, N);

x_init = [5 7 pi];
x = x_init;
x_desired = [0, 3, 0];
t_list = [];
x_list = [x];
steer_list = [];
for time = 0:0.1:20
    t_span = [0 0.1];
    x_error = x-x_desired;
    u = -K*x_error(2:end)';
    steer_list = [steer_list; u];
    dynamics_u = [v_trim, u];
    [t, x] = ode45(@(t, x) car_kinematics(t, x, dynamics_u), t_span , x);
    x = x(end,:);
    t_list = [t_list, time];
    x_list = [x_list; x];
end
% figure();
% plot(t_list, x_list(:, 2));
% figure();
% plot(t_list, steer_list);
figure()
plot(x_list(:,1), x_list(:,2));