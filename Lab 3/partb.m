%% LAB 3(b): OUTER-LOOP LEAD CONTROLLER DISCRETIZATION & DIFFERENCE EQUATION
clear; clc;
format long;

%% 1. Continuous-Time Outer-Loop Lead Controller C_2LD(s)
% C_2LD(s) = -7 * (s + 0.35) / (s + 2.5) = (-7*s - 2.45) / (s + 2.5)
% Input:  e_y(t) = y_ref(t) - y(t)  [m]
% Output: theta_ref(t)              [rad]
s = tf('s');
C2_LD = -7 * (s + 0.35) / (s + 2.5);

%% 2. Sampling Period (matches inner-loop 2 ms control ISR)
Ts = 0.002; % [s] (500 Hz)

%% 3. Analytical Tustin (Bilinear) Discretization
% Substitution: s = (2 / Ts) * (z - 1) / (z + 1)
k = 2 / Ts; % 1000.0

% Substitute into Num(s) = -7*s - 2.45 and Den(s) = s + 2.5,
% then multiply numerator and denominator by (z + 1):
%   Num(z) = (-7*k - 2.45)*z + (7*k - 2.45)
%   Den(z) = (k + 2.5)*z     + (-k + 2.5)
n1 = -7.0 * k - 2.45; % -7002.45
n0 =  7.0 * k - 2.45; %  6997.55
d1 =  k + 2.5;        %  1002.5
d0 = -k + 2.5;        %  -997.5

% Normalize denominator to monic form: D_2LD(z) = (b1*z + b0) / (a1*z + a0)
b1_outer = n1 / d1;
b0_outer = n0 / d1;
a1_outer = 1.0;
a0_outer = d0 / d1;

%% 4. Cross-Check with MATLAB c2d(..., 'tustin') & tfdata
D2_LD = c2d(C2_LD, Ts, 'tustin');
[numD2, denD2] = tfdata(D2_LD, 'v');

%% 5. Gain & Numerical Verification
dc_gain_ct = -7.0 * 0.35 / 2.5;
dc_gain_dt = (b1_outer + b0_outer) / (a1_outer + a0_outer);
hf_gain_dt = (b1_outer - b0_outer) / (a1_outer - a0_outer);
max_coeff_diff = max(abs([b1_outer, b0_outer, a1_outer, a0_outer] - [numD2, denD2]));

%% 6. Print Full 15-Decimal Results & Difference Equation
fprintf('=== Lab 3(b) Outer-Loop Lead Controller Discretization (Tustin, Ts = %.4f s) ===\n\n', Ts);

fprintf('Unnormalized D_2LD(z):\n');
fprintf('  (%.15f z + %.15f) / (%.15f z + %.15f)\n\n', n1, n0, d1, d0);

fprintf('Normalized D_2LD(z) = (b1_outer*z + b0_outer) / (a1_outer*z + a0_outer):\n');
fprintf('  b1_outer = %.15f\n', b1_outer);
fprintf('  b0_outer = %.15f\n', b0_outer);
fprintf('  a1_outer = %.15f\n', a1_outer);
fprintf('  a0_outer = %.15f\n\n', a0_outer);

fprintf('MATLAB c2d / tfdata cross-check (max coeff diff = %.3e):\n', max_coeff_diff);
fprintf('  numD2 = [%.15f, %.15f]\n', numD2(1), numD2(2));
fprintf('  denD2 = [%.15f, %.15f]\n\n', denD2(1), denD2(2));

fprintf('Gain checks:\n');
fprintf('  DC gain (z =  1): %.15f  (expected %.15f)\n', dc_gain_dt, dc_gain_ct);
fprintf('  HF gain (z = -1): %.15f  (expected -7.000000000000000)\n\n', hf_gain_dt);

fprintf('Difference equation (unsaturated state theta_ref[k]):\n');
fprintf('  theta_ref[k] = (%.15f)*e_y[k] + (%.15f)*e_y[k-1] - (%.15f)*theta_ref[k-1]\n', ...
    b1_outer, b0_outer, a0_outer);
fprintf('  theta_ref_sat[k] = min(max(theta_ref[k], -0.7), 0.7)\n');
