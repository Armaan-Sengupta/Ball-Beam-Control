%% LAB 2: INTERACTIVE POLE-PLACEMENT TUNER (NUMERICAL INPUTS & SLIDERS)
clear; clc; close all;

% 1. Plant Identification Parameters (SRV-02 plant)
plant.K1  = -2.053;
plant.tau = 0.02367;

% 2. Create Figure Window
hFig = figure('Name', 'Lab 2: Interactive Pole Placement Tuner', ...
              'NumberTitle', 'off', ...
              'Position', [40, 30, 1260, 750], ...
              'Color', [0.15 0.15 0.15]);

% 3. Create Axes
handles.ax_pz   = subplot(2, 2, 1, 'Parent', hFig); grid(handles.ax_pz, 'on'); hold(handles.ax_pz, 'on');
handles.ax_step = subplot(2, 2, 2, 'Parent', hFig); grid(handles.ax_step, 'on'); hold(handles.ax_step, 'on');
handles.ax_volt = subplot(2, 2, 4, 'Parent', hFig); grid(handles.ax_volt, 'on'); hold(handles.ax_volt, 'on');

set([handles.ax_pz, handles.ax_step, handles.ax_volt], ...
    'Color', [0.2 0.2 0.2], 'XColor', [0.9 0.9 0.9], 'YColor', [0.9 0.9 0.9], 'GridColor', [0.4 0.4 0.4]);

% 4. Mode Selection (Radio Button Group)
handles.bg = uibuttongroup('Parent', hFig, 'Position', [0.03, 0.44, 0.43, 0.055], ...
                           'BackgroundColor', [0.2 0.2 0.2], 'ForegroundColor', 'w', ...
                           'Title', 'Active Tuning Mode', 'HighlightColor', [0.5 0.5 0.5]);

handles.rb_sigma = uicontrol('Parent', handles.bg, 'Style', 'radiobutton', ...
                             'String', 'Real & Imag (\sigma, \omega_d)', ...
                             'Position', [15, 5, 200, 20], ...
                             'BackgroundColor', [0.2 0.2 0.2], 'ForegroundColor', 'w');

handles.rb_zeta  = uicontrol('Parent', handles.bg, 'Style', 'radiobutton', ...
                             'String', 'Damping & Nat Freq (\zeta, \omega_n)', ...
                             'Position', [225, 5, 240, 20], ...
                             'BackgroundColor', [0.2 0.2 0.2], 'ForegroundColor', 'w');

% 5. UI Sliders, Labels, and Exact Value Edit Boxes
% --- Parameter 1: Sigma or Zeta ---
handles.lbl_p1 = uicontrol('Parent', hFig, 'Style', 'text', 'String', 'Dominant Re (\sigma):', ...
                           'Position', [40, 280, 150, 18], 'HorizontalAlignment', 'left', ...
                           'BackgroundColor', [0.15 0.15 0.15], 'ForegroundColor', 'w');
handles.sld_p1 = uicontrol('Parent', hFig, 'Style', 'slider', ...
                           'Min', -35, 'Max', -8, 'Value', -16.0, ...
                           'Position', [40, 255, 180, 20]);
handles.edt_p1 = uicontrol('Parent', hFig, 'Style', 'edit', 'String', '-16.0', ...
                           'Position', [225, 255, 60, 22], ...
                           'BackgroundColor', [0.25 0.25 0.25], 'ForegroundColor', 'c', 'FontWeight', 'bold');

% --- Parameter 2: Omega_d or Omega_n ---
handles.lbl_p2 = uicontrol('Parent', hFig, 'Style', 'text', 'String', 'Dominant Im (\omega_d):', ...
                           'Position', [40, 225, 150, 18], 'HorizontalAlignment', 'left', ...
                           'BackgroundColor', [0.15 0.15 0.15], 'ForegroundColor', 'w');
handles.sld_p2 = uicontrol('Parent', hFig, 'Style', 'slider', ...
                           'Min', 0.1, 'Max', 30, 'Value', 8.5, ...
                           'Position', [40, 200, 180, 20]);
handles.edt_p2 = uicontrol('Parent', hFig, 'Style', 'edit', 'String', '8.5', ...
                           'Position', [225, 200, 60, 22], ...
                           'BackgroundColor', [0.25 0.25 0.25], 'ForegroundColor', 'c', 'FontWeight', 'bold');

% --- Non-Dominant Real Pole (p_3) ---
uicontrol('Parent', hFig, 'Style', 'text', 'String', 'Non-Dom Pole (p_3):', ...
          'Position', [40, 170, 150, 18], 'HorizontalAlignment', 'left', ...
          'BackgroundColor', [0.15 0.15 0.15], 'ForegroundColor', 'w');
handles.sld_nondom = uicontrol('Parent', hFig, 'Style', 'slider', ...
                               'Min', -90, 'Max', -15, 'Value', -48.0, ...
                               'Position', [40, 145, 180, 20]);
handles.edt_nondom = uicontrol('Parent', hFig, 'Style', 'edit', 'String', '-48.0', ...
                               'Position', [225, 145, 60, 22], ...
                               'BackgroundColor', [0.25 0.25 0.25], 'ForegroundColor', 'c', 'FontWeight', 'bold');

% Text readout for pole coordinates and controller gains
handles.txt_info = uicontrol('Parent', hFig, 'Style', 'text', ...
                             'Position', [40, 8, 540, 125], ...
                             'HorizontalAlignment', 'left', ...
                             'FontName', 'Courier', 'FontSize', 9, ...
                             'BackgroundColor', [0.2 0.2 0.2], ...
                             'ForegroundColor', [0.4 1 0.4]);

% Internal mathematical state
handles.mode    = 'sigma';
handles.sigma   = -16.0;
handles.omega_d = 8.5;
handles.zeta    = 16.0 / sqrt(16.0^2 + 8.5^2);
handles.omega_n = sqrt(16.0^2 + 8.5^2);
handles.p3      = -48.0;

handles.plant = plant;
guidata(hFig, handles);

% Bind Callbacks
set(handles.bg,         'SelectionChangedFcn', @modeSwitchCallback);
set(handles.sld_p1,     'Callback', @sliderCallback);
set(handles.sld_p2,     'Callback', @sliderCallback);
set(handles.sld_nondom, 'Callback', @sliderCallback);
set(handles.edt_p1,     'Callback', @editBoxCallback);
set(handles.edt_p2,     'Callback', @editBoxCallback);
set(handles.edt_nondom, 'Callback', @editBoxCallback);

% Initial render
updateSimulation(hFig);

%% Mode Switch Callback
function modeSwitchCallback(src, ~)
    hFig = ancestor(src, 'figure');
    handles = guidata(hFig);
    
    if get(handles.rb_sigma, 'Value') == 1
        handles.mode = 'sigma';
        set(handles.lbl_p1, 'String', 'Dominant Re (\sigma):');
        set(handles.sld_p1, 'Min', -40, 'Max', -5, 'Value', handles.sigma);
        set(handles.edt_p1, 'String', sprintf('%.2f', handles.sigma));
        
        set(handles.lbl_p2, 'String', 'Dominant Im (\omega_d):');
        set(handles.sld_p2, 'Min', 0.1, 'Max', 30, 'Value', handles.omega_d);
        set(handles.edt_p2, 'String', sprintf('%.2f', handles.omega_d));
    else
        handles.mode = 'zeta';
        set(handles.lbl_p1, 'String', 'Damping Ratio (\zeta):');
        set(handles.sld_p1, 'Min', 0.40, 'Max', 0.99, 'Value', min(0.99, max(0.40, handles.zeta)));
        set(handles.edt_p1, 'String', sprintf('%.3f', handles.zeta));
        
        set(handles.lbl_p2, 'String', 'Nat. Freq (\omega_n):');
        set(handles.sld_p2, 'Min', 10, 'Max', 50, 'Value', handles.omega_n);
        set(handles.edt_p2, 'String', sprintf('%.2f', handles.omega_n));
    end
    guidata(hFig, handles);
    updateSimulation(hFig);
end

%% Slider Callback
function sliderCallback(src, ~)
    hFig = ancestor(src, 'figure');
    handles = guidata(hFig);
    
    val1 = get(handles.sld_p1, 'Value');
    val2 = get(handles.sld_p2, 'Value');
    handles.p3 = get(handles.sld_nondom, 'Value');
    set(handles.edt_nondom, 'String', sprintf('%.2f', handles.p3));
    
    if strcmp(handles.mode, 'sigma')
        handles.sigma   = val1;
        handles.omega_d = val2;
        handles.omega_n = sqrt(handles.sigma^2 + handles.omega_d^2);
        handles.zeta    = abs(handles.sigma) / handles.omega_n;
        set(handles.edt_p1, 'String', sprintf('%.2f', handles.sigma));
        set(handles.edt_p2, 'String', sprintf('%.2f', handles.omega_d));
    else
        handles.zeta    = val1;
        handles.omega_n = val2;
        handles.sigma   = -handles.zeta * handles.omega_n;
        handles.omega_d = handles.omega_n * sqrt(max(0, 1 - handles.zeta^2));
        set(handles.edt_p1, 'String', sprintf('%.3f', handles.zeta));
        set(handles.edt_p2, 'String', sprintf('%.2f', handles.omega_n));
    end
    
    guidata(hFig, handles);
    updateSimulation(hFig);
end

%% Exact Edit Box Callback
function editBoxCallback(src, ~)
    hFig = ancestor(src, 'figure');
    handles = guidata(hFig);
    
    val1 = str2double(get(handles.edt_p1, 'String'));
    val2 = str2double(get(handles.edt_p2, 'String'));
    val3 = str2double(get(handles.edt_nondom, 'String'));
    
    if ~isnan(val3)
        handles.p3 = val3;
        % Auto-expand slider range if typed value is outside bounds
        if handles.p3 < get(handles.sld_nondom, 'Min'), set(handles.sld_nondom, 'Min', handles.p3 - 10); end
        if handles.p3 > get(handles.sld_nondom, 'Max'), set(handles.sld_nondom, 'Max', handles.p3 + 10); end
        set(handles.sld_nondom, 'Value', handles.p3);
    end
    
    if strcmp(handles.mode, 'sigma')
        if ~isnan(val1), handles.sigma = val1; end
        if ~isnan(val2), handles.omega_d = abs(val2); end
        
        handles.omega_n = sqrt(handles.sigma^2 + handles.omega_d^2);
        handles.zeta    = abs(handles.sigma) / handles.omega_n;
        
        if handles.sigma < get(handles.sld_p1, 'Min'), set(handles.sld_p1, 'Min', handles.sigma - 5); end
        if handles.sigma > get(handles.sld_p1, 'Max'), set(handles.sld_p1, 'Max', handles.sigma + 5); end
        if handles.omega_d > get(handles.sld_p2, 'Max'), set(handles.sld_p2, 'Max', handles.omega_d + 5); end
        
        set(handles.sld_p1, 'Value', handles.sigma);
        set(handles.sld_p2, 'Value', handles.omega_d);
    else
        if ~isnan(val1), handles.zeta = min(0.999, max(0.01, val1)); end
        if ~isnan(val2), handles.omega_n = max(0.1, val2); end
        
        handles.sigma   = -handles.zeta * handles.omega_n;
        handles.omega_d = handles.omega_n * sqrt(max(0, 1 - handles.zeta^2));
        
        if handles.omega_n > get(handles.sld_p2, 'Max'), set(handles.sld_p2, 'Max', handles.omega_n + 10); end
        
        set(handles.sld_p1, 'Value', handles.zeta);
        set(handles.sld_p2, 'Value', handles.omega_n);
    end
    
    guidata(hFig, handles);
    updateSimulation(hFig);
end

%% Simulation & Rendering Engine
function updateSimulation(hFig)
    handles = guidata(hFig);
    
    K1  = handles.plant.K1;
    tau = handles.plant.tau;
    
    sigma   = handles.sigma;
    omega_d = handles.omega_d;
    p3      = handles.p3;

    % Desired closed-loop poles
    p_dom1 = sigma + 1j*omega_d;
    p_dom2 = sigma - 1j*omega_d;
    desired_poles = [p_dom1, p_dom2, p3];

    % Polynomial matching
    d = real(poly(desired_poles));
    d2 = d(2); d1 = d(3); d0 = d(4);

    a1 = 1;
    a0 = d2 - (1/tau);
    b1 = (tau*d1 - a0) / K1;
    b0 = (tau*d0) / K1;

    c_pole = -a0;
    c_zero = -b0 / b1;
    p_motor_pole = -1 / tau;

    % Continuous transfer functions
    s = tf('s');
    P  = K1 / (s * (tau*s + 1));
    C1 = tf([b1, b0], [a1, a0]);
    T_cl = feedback(C1 * P, 1);
    S_u  = feedback(C1, P);

    % Step simulation (1.4 rad step)
    t = linspace(0, 0.6, 600);
    [y, ~] = step(1.4 * T_cl, t);
    [v, ~] = step(1.4 * S_u, t);

    sinfo = stepinfo(1.4 * T_cl);
    max_volt = max(abs(v));

    % --- 1. S-Plane Map ---
    cla(handles.ax_pz);
    plot(handles.ax_pz, real(desired_poles), imag(desired_poles), 'ms', ...
         'MarkerSize', 8, 'MarkerFaceColor', 'm', 'DisplayName', 'CL Poles');
     
    text(handles.ax_pz, real(p_dom1) + 1.5, imag(p_dom1), ...
         sprintf(' p_{1,2} = %.2f \\pm %.2fj', real(p_dom1), abs(imag(p_dom1))), ...
         'Color', 'm', 'FontSize', 8, 'FontWeight', 'bold');
    text(handles.ax_pz, p3 + 1.5, 2.5, ...
         sprintf(' p_3 = %.2f', p3), ...
         'Color', 'm', 'FontSize', 8, 'FontWeight', 'bold');

    plot(handles.ax_pz, c_zero, 0, 'ro', 'MarkerSize', 7, 'LineWidth', 1.5, 'DisplayName', 'C_1 Zero');
    text(handles.ax_pz, c_zero + 1.5, -4, sprintf(' z_c = %.2f', c_zero), 'Color', [1 0.4 0.4], 'FontSize', 8);

    plot(handles.ax_pz, c_pole, 0, 'rx', 'MarkerSize', 9, 'LineWidth', 1.5, 'DisplayName', 'C_1 Pole');
    text(handles.ax_pz, c_pole + 1.5, 4, sprintf(' p_c = %.2f', c_pole), 'Color', [1 0.4 0.4], 'FontSize', 8);

    plot(handles.ax_pz, [0, p_motor_pole], [0, 0], 'cx', 'MarkerSize', 9, 'LineWidth', 2, 'DisplayName', 'Plant Poles');
    text(handles.ax_pz, p_motor_pole + 1.5, -4, sprintf(' -1/\\tau = %.2f', p_motor_pole), 'Color', 'c', 'FontSize', 8);
    text(handles.ax_pz, 0 + 1.5, 3, ' 0', 'Color', 'c', 'FontSize', 8);

    xline(handles.ax_pz, -16, 'g--', 'Ts \le 0.25s Sim', 'Color', [0.3 0.8 0.3]);
    xline(handles.ax_pz, -10, 'y--', 'Ts \le 0.40s HW', 'Color', [0.8 0.8 0.2]);

    title(handles.ax_pz, 'Pole-Zero Locations (s-plane)', 'Color', 'w');
    xlabel(handles.ax_pz, 'Real (\sigma)', 'Color', 'w');
    ylabel(handles.ax_pz, 'Imag (j\omega)', 'Color', 'w');
    legend(handles.ax_pz, 'Location', 'southwest', 'TextColor', 'w', 'Color', [0.2 0.2 0.2]);
    
    x_min = min([p3, c_pole, c_zero, p_motor_pole]) - 15;
    xlim(handles.ax_pz, [min(x_min, -100), 10]);
    ylim(handles.ax_pz, [-max(30, omega_d+5), max(30, omega_d+5)]);
    grid(handles.ax_pz, 'on');

    % --- 2. Step Response Plot ---
    cla(handles.ax_step);
    plot(handles.ax_step, t, y, 'c', 'LineWidth', 1.8);
    yline(handles.ax_step, 1.4, 'w:');
    yline(handles.ax_step, 1.4 * 1.05, 'r--', '5% OS Spec');
    title(handles.ax_step, sprintf('\\theta(t) | Ts = %.3fs | OS = %.1f%%', ...
          sinfo.SettlingTime, sinfo.Overshoot), 'Color', 'w');
    xlabel(handles.ax_step, 'Time (s)', 'Color', 'w');
    ylabel(handles.ax_step, '\theta (rad)', 'Color', 'w');
    ylim(handles.ax_step, [0, 1.6]);
    grid(handles.ax_step, 'on');

    % --- 3. Control Effort (Voltage) Plot ---
    cla(handles.ax_volt);
    plot(handles.ax_volt, t, v, 'm', 'LineWidth', 1.8);
    yline(handles.ax_volt, 6, 'r--', '+6V Limit');
    yline(handles.ax_volt, -6, 'r--', '-6V Limit');
    title(handles.ax_volt, sprintf('Control Effort V(t) | Peak = %.2f V', max_volt), 'Color', 'w');
    xlabel(handles.ax_volt, 'Time (s)', 'Color', 'w');
    ylabel(handles.ax_volt, 'Voltage (V)', 'Color', 'w');
    ylim(handles.ax_volt, [-max(8, max_volt+1), 2]);
    grid(handles.ax_volt, 'on');

    % --- 4. Detailed Status Box ---
    status_v = 'PASS'; if max_volt > 6.0, status_v = 'FAIL'; end
    status_t = 'PASS'; if sinfo.SettlingTime > 0.25, status_t = 'FAIL'; end
    status_o = 'PASS'; if sinfo.Overshoot > 5.0, status_o = 'FAIL'; end

    info_str = sprintf([...
        'CL Dominant: p1,2 = %-12s | Non-Dom: p3 = %.2f\n' ...
        'Damping:     \\zeta = %.3f             | NatFreq: \\omega_n = %.2f rad/s\n' ...
        'C1(s) Pole:  pc = %.2f           | C1(s) Zero: zc = %.2f\n' ...
        'C1(s) = (%.3f s + %.3f) / (s + %.3f)\n' ...
        'Ts (2%%)      = %.3f s [%s]  (Sim Spec <= 0.25 s)\n' ...
        'Overshoot    = %.1f %%  [%s]  (Spec <= 5.0 %%)\n' ...
        'Peak |V(t)|  = %.2f V [%s]  (Limit <= 6.0 V)'], ...
        sprintf('%.2f ± %.2fj', real(p_dom1), abs(imag(p_dom1))), p3, ...
        handles.zeta, handles.omega_n, ...
        c_pole, c_zero, ...
        b1, b0, a0, ...
        sinfo.SettlingTime, status_t, ...
        sinfo.Overshoot, status_o, ...
        max_volt, status_v);

    set(handles.txt_info, 'String', info_str);
end