% x = [x y psi(yaw)] u = [v delta]
function dxdt = car_kinematics(t,x,u)
    dxdt = zeros(3,1);
    psi = x(3);
    v = u(1);
    delta = min(max(u(2), -1.22), 1.22);
    lr = 1.5;
    lf = 1;
%     center slip
    lamda = atan(lr*tan(delta)/(lr+lf));
    dxdt(1) = v*cos(psi + lamda);
    dxdt(2) = v*sin(psi + lamda);
    dxdt(3) = v*cos(lamda)*tan(delta)/(lr+lf);
end


