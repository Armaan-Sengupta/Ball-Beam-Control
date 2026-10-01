clc;

% Found Experimentally (2ms)
o_s = 0.3; 
t_p = 0.082;

% Constants
kp = 35;
theta_max = pi/4;
theta_min = -pi/4;

zeta = (-log(o_s))/(sqrt((pi^2) + (log(o_s)^2)))
omega = (pi)/(t_p * (sqrt(1 - (zeta^2))))
tau = 1/(2 * zeta * omega)
k1 = (tau * (omega^2))/kp
