% kf coefficient for force
% km coefficient for moment
function [Force, Moment] = propellerDynamics(kf, km, angularVel)
    Force = kf*angularVel^2;
    Moment = km*angularVel^2;
end