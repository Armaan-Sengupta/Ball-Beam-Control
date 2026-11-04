clc; clear all;

% Found Experimentally (2ms)
o_s = 0.3; 
t_p = 0.082;
kp = 35;

% Constants
theta_max = pi/4;
theta_min = -pi/4;

% Plant constants
zeta = (-log(o_s))/(sqrt((pi^2) + (log(o_s)^2))); % Helper
omega = (pi)/(t_p * (sqrt(1 - (zeta^2)))); % Helper
tau = 1/(2 * zeta * omega);
k1 = -(tau * (omega^2))/kp;


% Controller constants
b1 =    -2.597804;
b0 =  -225.747004;
a1 =     1.000000;
a0 =    44.752429;

% Discrete Controller constants
b1_d =    -2.702602561764911;
b0_d =    2.270448497190859;
a1_d =     1.000000;
a0_d =    -0.914329121458805;

set_param('lab_2/Discrete Transfer Fcn', 'SampleTime', '0.002')