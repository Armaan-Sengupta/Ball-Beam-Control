%% LAB 2: INNER-LOOP CONTROLLER DESIGN (POLE PLACEMENT & TUNING)
clear; clc; close all;

%% 1. Plant Identification Parameters (From Image)
% Plant: P(s) = K1 / (s * (tau*s + 1))
K1  = -2.053;       % rad/(V*s)
tau = 0.02367;      % s

% Define Plant Transfer Function
s = tf('s');
P = K1 / (s * (tau*s + 1));

fprintf('=== PLANT MODEL ===\n');


%% 2. Desired Closed-Loop Poles
% Change these to test different pole configurations:
p_dom1 = -16 + 10j;     % Dominant pole 1
p_dom2 = -16 - 10j;     % Dominant pole 2
p_nondom = -55;         % Non-dominant real pole

desired_poles = [p_dom1, p_dom2, p_nondom];

%% 3. Closed-Loop Characteristic Polynomial Matching
% Desired monic polynomial: Delta(s) = (s - p1)(s - p2)(s - p3)
% = s^3 + d2*s^2 + d1*s + d0
poly_des = real(poly(desired_poles));
d2 = poly_des(2);
d1 = poly_des(3);
d0 = poly_des(4);

% Closed-loop characteristic polynomial with C(s) = (b1*s + b0)/(a1*s + a0):
% Let a1 = 1 (monic controller denominator):
% Loop den: tau*s^3 + (1 + tau*a0)*s^2 + (a0 + K1*b1)*s + K1*b0 = 0
% Monic form: s^3 + (a0 + 1/tau)*s^2 + ((a0 + K1*b1)/tau)*s + (K1*b0)/tau = 0

a1 = 1;
a0 = d2 - (1/tau);
b1 = (tau*d1 - a0) / K1;
b0 = (tau*d0) / K1;

fprintf('=== COMPUTED CONTROLLER COEFFICIENTS ===\n');
fprintf('C(s) = (b1*s + b0) / (a1*s + a0)\n');
fprintf('  a1 = %12.6f\n', a1);
fprintf('  a0 = %12.6f\n', a0);
fprintf('  b1 = %12.6f\n', b1);
fprintf('  b0 = %12.6f\n\n', b0);

% Check Initial Voltage Limit: |v(0+)| = |b1 * 1.4 rad| <= 6 V
V_init = abs(b1 * 1.4);
fprintf('Max Initial Voltage for 1.4 rad step: |V(0+)| = %.3f V (Limit: 6.0 V)\n\n', V_init);

% Construct Controller
C1 = tf([b1 b0], [a1 a0]);

%% 4. Closed-Loop Transfer Functions & Performance
% Feedback returns a transfer function
theta_CL_TF = feedback(C1 * P, 1);     % Closed loop: Theta / Theta_ref
v_CL_TF  = feedback(C1, P);            % Control effort: V / Theta_ref

% Verify specs for 1.4 rad step
step_info = stepinfo(1.4 * theta_CL_TF);
step_info_v = stepinfo(1.4 * v_CL_TF);
fprintf('=== STEP RESPONSE SPECS (Continuous Simulation) ===\n');
fprintf('  Settling Time (2%%): %.4f s (Spec: <= 0.25 s)\n', step_info.SettlingTime);
fprintf('  Overshoot:         %.2f %%  (Spec: <= 5.0%%)\n', step_info.Overshoot);
fprintf('  Max voltage:         %.2f V  (Spec: <= 6V)\n', step_info_v.Peak);

%% 5. Discretize controller and close the loop
T = 0.002; % Sampling period (s)
D1 = c2d(C1, T, 'tustin');   % discrete controller
Pd = c2d(P, T, 'zoh');       % plant driven by a held voltage
[numD, denD] = tfdata(D1, 'v');
fprintf('=== DISCRETE CONTROLLER D1(z) (Sampling Time T = %.4f s) ===\n', T);
D1
for k = 1:numel(numD)
    fprintf('  num z^%d = %.15f\n', numel(numD) - k, numD(k));
end
for k = 1:numel(denD)
    fprintf('  den z^%d = %.15f\n', numel(denD) - k, denD(k));
end

theta_CL_TF_d = feedback(D1 * Pd, 1);   % Closed loop: Theta / Theta_ref
v_CL_TF_d     = feedback(D1, Pd);       % Control effort: V / Theta_ref

step_info_d = stepinfo(1.4 * theta_CL_TF_d);
step_info_v_d = stepinfo(1.4 * v_CL_TF_d);
fprintf('=== STEP RESPONSE SPECS (Discrete Simulation) ===\n');
fprintf('  Settling Time (2%%): %.4f s (Spec: <= 0.25 s)\n', step_info_d.SettlingTime);
fprintf('  Overshoot:         %.2f %%  (Spec: <= 5.0%%)\n', step_info_d.Overshoot);
fprintf('  Max voltage:         %.2f V  (Spec: <= 6V)\n', step_info_v_d.Peak);



%% 6. Plotting Verification
figure('Name', 'Continuous Closed-Loop Response');
subplot(2,1,1);
step(1.4 * theta_CL_TF, 0.6);
grid on;
title('Gear Angle Response \theta(t) for \Delta\theta = 1.4 rad');
ylabel('\theta (rad)');

subplot(2,1,2);
step(1.4 * v_CL_TF, 0.6);
grid on;
title('Control Effort V(t)');
ylabel('Voltage (V)');
xlabel('Time (s)');
yline(6, 'r--', 'Upper Limit (+6V)');
yline(-6, 'r--', 'Lower Limit (-6V)');


figure('Name', 'Discrete Closed-Loop Response');
subplot(2,1,1);
step(1.4 * theta_CL_TF_d, 0.6);
grid on;
title('Gear Angle Response \theta(t) for \Delta\theta = 1.4 rad');
ylabel('\theta (rad)');

subplot(2,1,2);
step(1.4 * v_CL_TF_d, 0.6);
grid on;
title('Control Effort V(t)');
ylabel('Voltage (V)');
xlabel('Time (s)');
yline(6, 'r--', 'Upper Limit (+6V)');
yline(-6, 'r--', 'Lower Limit (-6V)');
