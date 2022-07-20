% x = [x y psi psi_dot v_x v_y]
function dxdt = car_dynamics(t,x,u)


mass = 1600;
F_normal = mass/0.4; %normal force at each tire
CCf = 0.2; % cornering coefficient front
Cf = CCf * F_normal; % stiffness

end


