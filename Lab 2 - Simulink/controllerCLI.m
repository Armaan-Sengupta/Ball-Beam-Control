function result = controllerCLI(sigma, omega_d, p3)
%CONTROLLERCLI  Command-line Lab 2 pole-placement tuner (no GUI).
%
%   controllerCLI
%       Search sigma, omega_d, and p3 for designs that meet all three specs
%       on a 1.4 rad step:
%         Ts (2%) <= 0.25 s, overshoot <= 5%, peak |V(t)| <= 6 V
%
%   controllerCLI(sigma, omega_d, p3)
%       Evaluate one design and print the same metrics as controllerGUI.
%
%   Plant and gain formulas match controllerGUI.m:
%       P(s)  = K1 / (s * (tau*s + 1))
%       C1(s) = (b1*s + b0) / (s + a0)

    if nargin == 0
        result = searchDesigns();
    elseif nargin == 3
        result = evaluateDesign(sigma, omega_d, p3);
        printReport(result);
    else
        error('Usage: controllerCLI  or  controllerCLI(sigma, omega_d, p3)');
    end
end

function result = searchDesigns()
    fprintf('Lab 2 command-line pole search\n');
    fprintf('Specs for a 1.4 rad step: Ts(2%%) <= 0.25 s, OS <= 5%%, peak |V| <= 6 V\n\n');

    fprintf('GUI slider default (sigma = -16, omega_d = 8.5, p3 = -48)\n');
    baseline = evaluateDesign(-16, 8.5, -48);
    printReport(baseline);

    % Slider limits from controllerGUI.m.
    bounds.sigma = [-35, -8];
    bounds.wd    = [0.1, 25];
    bounds.p3    = [-80, -15];

    fprintf('Round 1: coarse grid\n');
    coarse = sweep(makeGrid(-34:2:-14, 1:2:17, -75:10:-25));
    printSweepSummary(coarse);

    hits = coarse([coarse.pass]);
    if isempty(hits)
        fprintf('Round 2: denser grid around the closest misses\n');
        [~, order] = sort([coarse.score], 'descend');
        seeds = coarse(order(1:min(4, numel(order))));
        refined = repmat(emptyResult(), 0, 1);
        for i = 1:numel(seeds)
            window = makeGrid( ...
                clipRange(seeds(i).sigma + (-4:1:4), bounds.sigma), ...
                clipRange(seeds(i).omega_d + (-4:1:4), bounds.wd), ...
                clipRange(seeds(i).p3 + (-20:5:20), bounds.p3));
            refined = [refined; sweep(window)]; %#ok<AGROW>
        end
        printSweepSummary(refined);
        pool = [coarse; refined];
    else
        fprintf('Round 2: refine the passing region\n');
        window = makeGrid( ...
            clipRange(min([hits.sigma])-2 : 1 : max([hits.sigma])+2, bounds.sigma), ...
            clipRange(min([hits.omega_d])-2 : 1 : max([hits.omega_d])+2, bounds.wd), ...
            clipRange(min([hits.p3])-10 : 5 : max([hits.p3])+10, bounds.p3));
        window = thinGrid(window, 700);
        refined = sweep(window);
        printSweepSummary(refined);
        pool = [coarse; refined];
    end

    hits = pool([pool.pass]);
    start = pickBest(pool);
    fprintf('\nRound 3: coordinate search from the best point so far\n');
    [tuned, trace] = coordinateSearch(start, bounds);
    printTrace(trace);

    extra = trace([trace.pass]);
    hits = [hits(:); extra(:)];
    hits = uniqueDesigns(hits);
    winner = pickBest([hits(:); tuned]);

    fprintf('\n========== SEARCH RESULT ==========\n');
    if isempty(hits)
        fprintf('No design on this search met all three specs.\n');
        fprintf('Closest point:\n');
        printReport(winner);
        result = winner;
        return;
    end

    fprintf('%d confirmed design(s) meet all three specs.\n', numel(hits));
    fprintf('Extremes of the passing set: sigma [%.1f, %.1f], omega_d [%.1f, %.1f], p3 [%.1f, %.1f]\n', ...
        min([hits.sigma]), max([hits.sigma]), ...
        min([hits.omega_d]), max([hits.omega_d]), ...
        min([hits.p3]), max([hits.p3]));
    fprintf('Use a listed design. A mix of those extremes is not necessarily a passing design.\n');

    fprintf('\nRecommended (largest simultaneous spec margin, stable C1 preferred):\n');
    printReport(winner);

    show = rankedHits(hits, 12);
    fprintf('Other working designs (best margins first):\n');
    fprintf('%-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s\n', ...
        'sigma', 'wd', 'p3', 'zeta', 'Ts', 'OS%', 'Vpk', 'margin');
    for i = 1:numel(show)
        h = show(i);
        fprintf('%-8.1f %-8.1f %-8.1f %-8.3f %-8.3f %-8.2f %-8.2f %-8.3f\n', ...
            h.sigma, h.omega_d, h.p3, h.zeta, h.Ts, h.OS, h.Vpeak, h.score);
    end
    result = winner;
end

function results = sweep(points)
    points = unique(round(points, 4), 'rows');
    n = size(points, 1);
    results = repmat(emptyResult(), n, 1);
    nPass = 0;
    bestScore = -inf;
    tStart = tic;
    fprintf('  evaluating %d points\n', n);
    for k = 1:n
        results(k) = evaluateDesign(points(k, 1), points(k, 2), points(k, 3));
        nPass = nPass + results(k).pass;
        bestScore = max(bestScore, results(k).score);
        if mod(k, 80) == 0 || k == n
            fprintf('  %d/%d  passes %d  best score %+.3f  (%.1fs)\n', ...
                k, n, nPass, bestScore, toc(tStart));
        end
    end
end

function printSweepSummary(results)
    nPass = sum([results.pass]);
    best = pickBest(results);
    fprintf('  summary: %d/%d passed. Best sigma=%.1f wd=%.1f p3=%.1f score %+.3f\n', ...
        nPass, numel(results), best.sigma, best.omega_d, best.p3, best.score);
end

function [best, trace] = coordinateSearch(start, bounds)
    best = start;
    trace = start(:);
    steps = [
        2.0, 2.0, 8.0;
        1.0, 1.0, 4.0;
        0.5, 0.5, 2.0;
        0.2, 0.2, 1.0];
    dirs = [
        -1  0  0;
         1  0  0;
         0 -1  0;
         0  1  0;
         0  0 -1;
         0  0  1];

    maxTrials = 120;
    for s = 1:size(steps, 1)
        improved = true;
        while improved && numel(trace) < maxTrials
            improved = false;
            for d = 1:size(dirs, 1)
                trialSigma = clip(best.sigma   + dirs(d, 1) * steps(s, 1), bounds.sigma);
                trialWd    = clip(best.omega_d + dirs(d, 2) * steps(s, 2), bounds.wd);
                trialP3    = clip(best.p3      + dirs(d, 3) * steps(s, 3), bounds.p3);
                if trialSigma == best.sigma && trialWd == best.omega_d && trialP3 == best.p3
                    continue;
                end
                trial = evaluateDesign(trialSigma, trialWd, trialP3);
                trace(end + 1, 1) = trial; %#ok<AGROW>
                fprintf('  try sigma=%6.1f wd=%5.1f p3=%6.1f | Ts=%s OS=%s V=%s\n', ...
                    trial.sigma, trial.omega_d, trial.p3, ...
                    specText(trial.Ts, '%.3f', trial.passT), ...
                    specText(trial.OS, '%.2f', trial.passO), ...
                    specText(trial.Vpeak, '%.2f', trial.passV));
                if trial.score > best.score + 1e-4
                    best = trial;
                    improved = true;
                end
            end
        end
    end
end

function printTrace(trace)
    nPass = sum([trace.pass]);
    fprintf('  coordinate search tried %d points, %d passed\n', numel(trace), nPass);
end

function best = pickBest(results)
    if isempty(results)
        best = emptyResult();
        return;
    end
    % Prefer a stable controller pole and a left-half-plane zero, then margin.
    rank = zeros(numel(results), 1);
    for i = 1:numel(results)
        stablePole = results(i).c_pole < 0;
        lhpZero = results(i).c_zero < 0;
        rank(i) = results(i).score + 0.02 * stablePole + 0.01 * lhpZero;
    end
    [~, idx] = max(rank);
    best = results(idx);
end

function hits = rankedHits(hits, limit)
    if isempty(hits)
        return;
    end
    rank = zeros(numel(hits), 1);
    for i = 1:numel(hits)
        stablePole = hits(i).c_pole < 0;
        lhpZero = hits(i).c_zero < 0;
        rank(i) = hits(i).score + 0.02 * stablePole + 0.01 * lhpZero;
    end
    [~, order] = sort(rank, 'descend');
    hits = hits(order);
    hits = hits(1:min(limit, numel(hits)));
end

function hits = uniqueDesigns(hits)
    if isempty(hits)
        return;
    end
    keys = round([[hits.sigma]', [hits.omega_d]', [hits.p3]'], 4);
    [~, ia] = unique(keys, 'rows', 'stable');
    hits = hits(ia);
end

function points = thinGrid(points, maxPoints)
    n = size(points, 1);
    if n <= maxPoints
        return;
    end
    idx = unique(round(linspace(1, n, maxPoints)));
    points = points(idx, :);
    fprintf('  thinned %d points down to %d\n', n, size(points, 1));
end

function points = makeGrid(sigmas, wds, p3s)
    sigmas = sigmas(:);
    wds = wds(:);
    p3s = p3s(:);
    [S, W, P] = ndgrid(sigmas, wds, p3s);
    points = [S(:), W(:), P(:)];
end

function values = clipRange(values, bounds)
    values = values(values >= bounds(1) & values <= bounds(2));
    if isempty(values)
        values = mean(bounds);
    end
end

function y = clip(y, bounds)
    y = min(bounds(2), max(bounds(1), y));
end

function r = evaluateDesign(sigma, omega_d, p3)
% Same plant, pole placement, step, and pass tests as controllerGUI.m.
    K1  = -2.053;
    tau = 0.02367;
    stepAmp = 1.4;

    r = emptyResult();
    r.sigma = sigma;
    r.omega_d = omega_d;
    r.p3 = p3;
    r.omega_n = hypot(sigma, omega_d);
    r.zeta = abs(sigma) / r.omega_n;

    wn2 = sigma^2 + omega_d^2;
    d2 = -2 * sigma - p3;
    d1 = wn2 + 2 * sigma * p3;
    d0 = -wn2 * p3;

    r.a1 = 1;
    r.a0 = d2 - (1 / tau);
    r.b1 = (tau * d1 - r.a0) / K1;
    r.b0 = (tau * d0) / K1;
    r.c_pole = -r.a0;
    if abs(r.b1) > 1e-9
        r.c_zero = -r.b0 / r.b1;
    else
        r.c_zero = NaN;
    end

    % |V(0+)| = |b1| * step amplitude. Already over the limit is a hard fail.
    r.V0 = r.b1 * stepAmp;
    if abs(r.V0) > 6
        r.Vpeak = abs(r.V0);
        r.passV = false;
        % Worse than any fully simulated miss, since |V(0+)| already breaks the spec.
        r.score = -2 - (r.Vpeak - 6) / 6;
        return;
    end

    den = [tau, (1 + tau * r.a0), (r.a0 + K1 * r.b1), K1 * r.b0];
    T_cl = tf(K1 * [r.b1, r.b0], den);
    S_u  = tf([r.b1 * tau, (r.b1 + r.b0 * tau), r.b0, 0], den);

    t = linspace(0, 0.6, 600);
    v = step(stepAmp * S_u, t);
    sinfo = stepinfo(stepAmp * T_cl, 'SettlingTimeThreshold', 0.02);

    r.Ts = sinfo.SettlingTime;
    r.OS = sinfo.Overshoot;
    if isfield(sinfo, 'Undershoot')
        r.undershoot = sinfo.Undershoot;
    end
    r.Vpeak = max(abs(v));

    r.passT = ~isnan(r.Ts) && r.Ts <= 0.25;
    r.passO = ~isnan(r.OS) && r.OS <= 5.0;
    r.passV = r.Vpeak <= 6.0;
    r.pass = r.passT && r.passO && r.passV;

    pen = 0;
    if ~r.passT
        if isnan(r.Ts)
            pen = pen + 5;
        else
            pen = pen + (r.Ts - 0.25) / 0.25;
        end
    end
    if ~r.passO
        pen = pen + max(0, r.OS - 5) / 5;
    end
    if ~r.passV
        pen = pen + (r.Vpeak - 6) / 6;
    end
    if r.pass
        r.score = min([(0.25 - r.Ts) / 0.25, (5 - r.OS) / 5, (6 - r.Vpeak) / 6]);
    else
        r.score = -pen;
    end
end

function r = emptyResult()
    r = struct( ...
        'sigma', NaN, 'omega_d', NaN, 'p3', NaN, ...
        'zeta', NaN, 'omega_n', NaN, ...
        'a1', NaN, 'a0', NaN, 'b1', NaN, 'b0', NaN, ...
        'c_pole', NaN, 'c_zero', NaN, ...
        'Ts', NaN, 'OS', NaN, 'undershoot', NaN, ...
        'V0', NaN, 'Vpeak', NaN, ...
        'passT', false, 'passO', false, 'passV', false, 'pass', false, ...
        'score', -inf);
end

function printReport(r)
    fprintf('  poles:     %.2f +/- %.2fj,  p3 = %.2f\n', r.sigma, r.omega_d, r.p3);
    fprintf('  damping:   zeta = %.3f,  wn = %.2f rad/s\n', r.zeta, r.omega_n);
    fprintf('  gains:     a1 = 1,  a0 = %.4f,  b1 = %.4f,  b0 = %.4f\n', r.a0, r.b1, r.b0);
    fprintf('  C1(s) = (%.4f*s %+.4f) / (s %+.4f)\n', r.b1, r.b0, r.a0);
    fprintf('  C1 pole = %.2f,  C1 zero = %.2f\n', r.c_pole, r.c_zero);
    fprintf('  closed-loop pole error = %.2e\n', poleError(r));
    fprintf('  Ts (2%%)   = %7.3f s  [%s]  spec <= 0.25 s\n', r.Ts, passFail(r.passT));
    fprintf('  Overshoot = %7.2f %%  [%s]  spec <= 5 %%\n', r.OS, passFail(r.passO));
    fprintf('  Peak |V|  = %7.2f V  [%s]  spec <= 6 V\n', r.Vpeak, passFail(r.passV));
    if ~isnan(r.undershoot)
        fprintf('  Undershoot= %7.2f %% (not part of the 3 specs)\n', r.undershoot);
    end
    if r.pass
        fprintf('  RESULT: PASS all 3 specs (margin score %.3f)\n\n', r.score);
    else
        fprintf('  RESULT: FAIL (score %+.3f)\n\n', r.score);
    end
end

function err = poleError(r)
    if isnan(r.a0) || isnan(r.b1)
        err = NaN;
        return;
    end
    K1 = -2.053;
    tau = 0.02367;
    den = [tau, (1 + tau * r.a0), (r.a0 + K1 * r.b1), K1 * r.b0];
    placed = roots(den);
    desired = [r.sigma + 1j * r.omega_d; r.sigma - 1j * r.omega_d; r.p3];
    err = 0;
    for k = 1:numel(desired)
        err = max(err, min(abs(placed - desired(k))));
    end
end

function label = passFail(ok)
    if ok
        label = 'PASS';
    else
        label = 'FAIL';
    end
end

function text = specText(value, fmt, ok)
    if isnan(value)
        body = '  nan';
    else
        body = sprintf(fmt, value);
    end
    text = sprintf(['%' num2str(max(6, numel(body))) 's %s'], body, passFail(ok));
end
