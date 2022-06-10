function [phi_command, theta_command, psi_command] = AcceerationToEuler(accx_desire, accy_desire, accz_desire, psi_desired)
    g = 9.806;
    eulerToAccMat = g*[sin(psi_desired) cos(psi_desired);...
                       -cos(psi_deesired) sin(psi_desired)];
    command_phi_theta = inv(eulerToAccMat)*[accx_desire;accy_desire];
    phi_command = command_phi_theta(1);
    theta_command = command_phi_theta(2);
    psi_command = psi_desired;
end