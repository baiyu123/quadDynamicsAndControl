prop_moment= [0.0040 0.0052 0.0066 0.0081 0.0095 0.0110 0.0132 0.0148...
    0.0171 0.0195 0.0226 0.0266 0.0293 0.0341 0.0361 0.0398];
prop_force = [0.150 0.194 0.238 0.294 0.350 0.421 0.513 0.608 0.666...
    0.778 0.868 0.989 1.079 1.180 1.304 1.467];
prop_breakpoints = [5 10 15 20 25 30 35 40 45 50 55 60 65 70 75 80];
mass = 0.481;
g = 9.806;
arm_length = 0.178;
inertia_matrix = [0.0034 0 0; 0 0.004 0; 0 0 0.0069];
% linearized relation between propforce and moment, moment = prop_force*(56.33/2013.8)-1.7/(1013.8)
kf = 56.33/(2013.8);

% initial condition
POS_INIT = [0 0 0];
VEL_INIT = [0 0 0];
EULER_INIT = [0 0 0];
ANG_VEL_INIT = [0 0 0];