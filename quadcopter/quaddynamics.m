% x = x y z u v w phi theta psi p q r
% input = F1 F2 F3 F4
% I inertia matrix
% m mass
% L quad arm length
% M moments of each propeller
function [pos_dot, v_dot, euler_dot, angularVel_dot] = quadDyanmics(pos, velocityBody, eulerAngle, angularVel, input, I, m, L, M)
    g = 9.806;
    F1 = input(1);
    F2 = input(2);
    F3 = input(3);
    F4 = input(4);
    phi = eulerAngle(1);
    theta = eulerAngle(2);
    psi = eulerAngle(3);
    Rz = [cos(psi) -sin(psi) 0;
          sin(psi) cos(psi)  0;
          0        0         1];
    Ry = [cos(theta)  0 sin(theta);
          0           1          0;
          -sin(theta) 0 cos(theta)];
    Rx = [1 0                0;
          0 cos(phi) -sin(phi);
          0 sin(phi)  cos(phi)];
    R = Rz*Ry*Rx;
%     differential equation
    pos_dot = R*velocityBody;
    v_dot = R'*[0 0 -g] + [0 0 (F1 + F2 + F3 + F4)/m]';
    angularVel_dot = inv(I)*([L*(F2-F4); ...
                              L(F3-F1); ...
                              M(1)-M(2)+M(3)-M(4)] ...
                              - cross(angularVel,I*angularVel));
    euler_dot = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
                 0 cos(phi)            -sin(phi);
                 0 sin(phi)*sec(theta) cos(phi)*sec(theta)]*angularVel_dot;
end
