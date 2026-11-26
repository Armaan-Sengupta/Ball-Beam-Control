clc;

tau = 0.02367
k1 = -2.0530

% controller values
n1 = -2.598
n0 = -225.747

d1 = 1 
d0 = 44.752

% k1 = 2.0530; 
% tau = 0.02367;      % use the values your tool used
% wn = 19; zeta = 0.85; p = 60;
% d0 = p + 2*zeta*wn - 1/tau;
% n1 = (tau*(wn^2 + 2*zeta*wn*p) - d0)/k1;
% n0 = tau*p*wn^2/k1;
% d1 = 1;

Ts = 0.002;
C1 = tf([n1 n0], [1 d0]);
D1 = c2d(C1, Ts, 'tustin');
[NUM, DEN] = tfdata(D1, 'v');

K2 = 0.0609;
K3 = 0.2632/K2;   % ≈ -4.32

y0   = 0.10;            % initial ball position [m]
C3 = tf(7*[1 0.35], [1 2.5]);   % sign flipped for our theta convention
C2 = tf(1*[57.94 56.99 23.75], [1 6.6 0]); % sign flipped for our theta convention
[C2_num, C2_den] = tfdata(C2, 'v');
[C3_num, C3_den] = tfdata(C3, 'v');
T_sq = 20;

D2 = c2d(C2, Ts, 'tustin')
 