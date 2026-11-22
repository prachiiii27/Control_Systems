%% ES 245 Task 4: measure the hardware results and compare with the model
% Reads the CSV logs made by ES245_Log_Serial.m (or pasted from the Serial
% Monitor, with timestamps switched OFF), measures every 10 cm step, and
% plots each test against what the linear model predicts.

clear; clc; close all;

%% 1) Settings
files = ["log_P.csv", "log_PD.csv", "log_PID.csv"];   % remove any you have not recorded
band  = 0.5;      % settling band: within +/-0.5 cm (5% of the 10 cm step)
avgN  = 5;        % average 5 samples (0.2 s) to smooth sensor noise before measuring

% Plant in Arduino units: cm of car position per degree of servo, per s^2
K  = 9.81*0.05/0.25 * (pi/180) * 100;    % = 3.424
Tf = 0.06;        % the Arduino's derivative filter is about 60 ms

% Gains used on the Arduino (deg/cm):  [Kp  Ki  Kd  b]
gains = [0.66 0    0    1.0;    % mode 1: P
         0.66 0    0.69 1.0;    % mode 2: PD
         1.38 0.46 0.97 0.7];   % mode 3: PID
names = ["P", "PD", "PID"];

s = tf('s');
G = K/s^2;

ctrl = strings(0,1); stepNo = []; stepSize = []; rise = []; OS = []; Ts = []; ess = []; umax = [];

%% 2) Go through each log
for f = 1:numel(files)
    if ~isfile(files(f)), fprintf('Skipping %s (not found)\n', files(f)); continue; end
    fid = fopen(files(f));
    C = textscan(fid, '%f %f %f %f %f', 'Delimiter', ',', 'CommentStyle', '#');
    fclose(fid);
    t = C{1}; r = C{2}; y = C{3}; u = C{4}; m = C{5};
    ok = ~isnan(t) & ~isnan(y) & ~isnan(m);
    t = t(ok); r = r(ok); y = y(ok); u = u(ok); m = m(ok);
    md = round(median(m));                     % 1, 2 or 3
    ys = movmean(y, avgN);                     % smoothed position

    % Model prediction for the same set point signal
    g  = gains(md,:);
    Cr = g(4)*g(1) + g(2)/s;                       % acts on the set point
    Cy = g(1) + g(2)/s + g(3)*s/(Tf*s + 1);        % acts on the measurement
    T  = minreal(Cr*feedback(G, Cy));
    tu = (t(1):0.01:t(end))';
    ru = interp1(t, r, tu, 'previous', 'extrap');
    ysim = y(1) + lsim(T, ru - y(1), tu - tu(1));

    % Measure every set point step
    idx = find(abs(diff(r)) > 1) + 1;
    for k = 1:numel(idx)
        i = idx(k);
        if k < numel(idx), j = idx(k+1) - 1; else, j = numel(t); end
        tt = t(i:j) - t(i);
        if tt(end) < 8, continue; end              % skip a step cut short by the end of the log
        yy = ys(i:j); r0 = r(i-1); r1 = r(i); S = r1 - r0;
        n  = (yy - r0)/S;                          % 0 at the old set point, 1 at the new one

        i10 = find(n >= 0.1, 1); i90 = find(n >= 0.9, 1);
        if isempty(i10) || isempty(i90) || n(1) >= 0.1, tr = NaN; else, tr = tt(i90) - tt(i10); end
        out = find(abs(yy - r1) > band, 1, 'last');
        if isempty(out), tsettle = 0; elseif out < numel(tt), tsettle = tt(out+1); else, tsettle = NaN; end

        ctrl(end+1,1)   = names(md);
        stepNo(end+1,1) = k;
        stepSize(end+1,1) = S;
        rise(end+1,1)   = tr;
        OS(end+1,1)     = max(0, (max(n) - 1)*100);
        Ts(end+1,1)     = tsettle;                          % NaN = never settled
        ess(end+1,1)    = mean(r1 - yy(tt > tt(end) - 2));  % average error over the last 2 s
        umax(end+1,1)   = max(abs(u(i:j)));
    end

    % Plot hardware against model
    figure('Name', names(md));
    plot(t, r, 'k--', 'LineWidth', 1.2); hold on;
    plot(t, y, 'Color', [0.2 0.4 0.8], 'LineWidth', 1.4);
    plot(tu, ysim, 'Color', [0.85 0.3 0.2], 'LineWidth', 1.2);
    grid on; xlabel('Time (s)'); ylabel('Sensor reading (cm)');
    legend('Set point', 'Hardware', 'Model', 'Location', 'best');
    title(names(md) + " controller: hardware vs model");
    exportgraphics(gcf, "hw_" + names(md) + ".png", 'Resolution', 200);
end

%% 3) Results
results = table(ctrl, stepNo, stepSize, rise, OS, Ts, ess, umax, ...
    'VariableNames', {'Controller','Step','Size_cm','Rise_s','Overshoot_pct', ...
                      'Settling_s','SSerror_cm','MaxServo_deg'});
disp('Every step:'); disp(results);
disp('Average per controller (NaN settling = never settled):');
disp(groupsummary(results, 'Controller', 'mean', ...
     {'Rise_s','Overshoot_pct','Settling_s','SSerror_cm','MaxServo_deg'}));
writetable(results, 'hardware_results.csv');
