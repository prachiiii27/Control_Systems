%% ES 245 Course Project Part 2 : Tasks 5, 6 and 7 from our 4 chirp trials
% Setup 5, Group 21.  Input u = servo angle from centre [deg], output y = car position [cm]
% Put this script in the same folder as the four .txt logs and press Run.
% Each section matches one task, so you can run them one at a time with "Run Section".

clear; clc; close all;
s = tf('s');

%% 0) Settings
files  = {'G21A20f11.txt','G21A25f11.txt','G21A20f12.5.txt','G21A25f11.5.txt'};
names  = {'Trial 1 (A=20, f1=1 Hz)','Trial 2 (A=25, f1=1 Hz)', ...
          'Trial 3 (A=20, f1=2.5 Hz)','Trial 4 (A=25, f1=1.5 Hz)'};
f1     = [1.0 1.0 2.5 1.5];     % chirp end frequency of each trial [Hz]
f0     = 0.1;  Tch = 20;        % chirp start frequency [Hz] and length [s]
servoC = 89;                    % servo centre used by the chirp code [deg]
tCut   = 10;                    % remove the first 10 s (10000 ms) of every file
Ts     = 0.02;                  % resampling period [s] (logger target was 20 ms)
Td     = 0.09;                  % delay used for robustness checks: 70 ms servo + 20 ms half sample [s]
K_nom  = 3.424;                 % Part 1 model gain [cm/(deg s^2)]

%% Task 5a) Load, cut the first 10 s, clean and resample every trial
data = cell(1,4);  raw = cell(1,4);
for i = 1:4
    txt = fileread(files{i});
    tok = regexp(txt, '(\d+),(-?[\d\.]+),(-?[\d\.]+)', 'tokens');
    M   = str2double(vertcat(tok{:}));          % columns: time_ms, servo_deg, dist_cm
    raw{i} = M;
    t = M(:,1)/1000;  u = M(:,2);  y = M(:,3);
    keep = (t >= tCut) & (y > 0);                % first 10 s out, -1 (no echo) out
    t = t(keep);  u = u(keep);  y = y(keep);
    med = movmedian(y, 7);                       % ghost echoes (about 5 cm) are single spikes
    bad = abs(y - med) > 2.5;
    y(bad) = med(bad);
    fprintf('%s: %d samples kept, %d spikes replaced\n', names{i}, numel(t), sum(bad));
    tu = (t(1):Ts:t(end))';
    u  = interp1(t, u, tu) - servoC;
    y  = interp1(t, y, tu);
    y  = y - mean(y);
    data{i} = iddata(y, u, Ts, 'InputName','theta','OutputName','a', ...
                     'InputUnit','deg','OutputUnit','cm','Tstart',tu(1));
end

figure('Name','Task 5 raw logs');
for i = 1:4
    subplot(4,1,i);
    M = raw{i};  ok = M(:,3) > 0;
    yyaxis left;  plot(M(ok,1)/1000, M(ok,3), '.', 'MarkerSize', 4); ylabel('a [cm]'); ylim([0 45]);
    yyaxis right; plot(M(:,1)/1000, M(:,2)); ylabel('\theta [deg]');
    xline(tCut, '--'); title(names{i}); grid on;
end
xlabel('t [s]');

%% Task 5b) Identify the transfer function with n4sid
% Trial 1 has the largest motion (about +/-4 cm, ten times the sensor noise) and the car
% never hit an end, so it is the identification data. Trials 2 to 4 are used to check it.
% Trial 2: the car hit the sensor end from 11.5 to 14.8 s, so its data is saturated.
dEst = data{1};

% Step 1: plain n4sid (what the brief asks for). Keep its output for the report.
opt  = n4sidOptions('Focus','simulation', 'InitialState','estimate');
sys_n4 = n4sid(dEst, 2, 'Ts', 0, opt);                     % 2nd order, continuous time
disp('n4sid model (unconstrained):');  tf(sys_n4)
disp('n4sid poles:');  disp(pole(sys_n4));
% On our data n4sid puts one pole near -1400 rad/s (far above the 157 rad/s Nyquist
% limit of 20 ms sampling) and adds a zero. That pole is fitting sensor noise, not physics.

% Step 2: same order, but with the physical structure of the plant:
% 2 poles, no zero (car on beam = force -> acceleration), started from the Part 1 model.
init = idtf(K_nom, [1 0 0]);                               % Part 1 model 3.424/s^2 as the start
optT = tfestOptions('InitialCondition','estimate', 'EnforceStability', false);
sys_id = tfest(dEst, init, optT);
G = tf(sys_id);  G = G(1,1);

disp('Identified model G(s), theta [deg] -> a [cm]:');  G
disp('Poles of G:');  disp(pole(G));
% Expected (our Python check of the same data): G(s) = 3.072/(s^2 + 1.243 s - 0.503),
% poles 0.322 and -1.564, fit about 65 %. If tfest still gives something very
% different, uncomment the next line:
% G = 3.0718/(s^2 + 1.2428*s - 0.5033);  sys_id = idtf(G);

figure('Name','n4sid vs structured');
compare(dEst, sys_n4, sys_id);  grid on;
legend('measured','n4sid (unconstrained)','2 poles, no zero');
title('Task 5: n4sid against the structured model');

figure('Name','Task 5 fit');
compare(dEst, sys_id);  grid on;  title('Task 5: Trial 1 measured vs identified model');
figure('Name','Task 5 validation');
for i = 2:4
    subplot(3,1,i-1);  compare(data{i}, sys_id);  grid on;  title(['Validation: ' names{i}]);
end

%% Task 5c) Bode plot of the model with the measured chirp points
figure('Name','Task 5 Bode');
w = logspace(-1, log10(30), 400);
[magG, phG] = bode(G, w);       magG = squeeze(magG);  phG = squeeze(phG);
[magN, phN] = bode(K_nom/s^2, w); magN = squeeze(magN); phN = squeeze(phN);
if phG(1) > 0, phG = phG - 360; end
subplot(2,1,1); semilogx(w, 20*log10(magG), 'k', w, 20*log10(magN), 'k--'); hold on; grid on;
subplot(2,1,2); semilogx(w, phG, 'k', w, phN, 'k--'); hold on; grid on;
mk = {'o','x','s','^'};
for i = 1:4
    P = chirpPoints(raw{i}, servoC, tCut, f0, f1(i), Tch);
    ph = P(:,3);  ph(ph > 0) = ph(ph > 0) - 360;
    subplot(2,1,1); semilogx(P(:,1), 20*log10(P(:,2)), ['k' mk{i}]);
    subplot(2,1,2); semilogx(P(:,1), ph, ['k' mk{i}]);
end
subplot(2,1,1); ylabel('Magnitude [dB]'); title('Task 5: Bode plot, model and measured chirp points');
legend([{'identified G','Part 1: 3.424/s^2'}, names], 'Location','southwest');
subplot(2,1,2); ylabel('Phase [deg]'); xlabel('\omega [rad/s]'); ylim([-300 -60]);

%% Task 6.1) Root locus and Nyquist
Gd = G*pade(exp(-Td*s), 3);
figure('Name','Task 6.1 root locus');
subplot(1,2,1); rlocus(G);  grid on; axis([-4 2 -6 6]);   title('Root locus of G(s)');
subplot(1,2,2); rlocus(Gd); grid on; axis([-8 4 -12 12]); title('Root locus of G(s) with 90 ms delay');
disp('Closed loop poles at K = 1:');  disp(pole(feedback(G,1)));
[GmG, ~] = margin(G);
fprintf('Smallest gain for stability (pole at %.3f must be pulled left): K > %.3f\n', max(real(pole(G))), GmG);

figure('Name','Task 6.1 Nyquist');
subplot(1,2,1); nyquist(G);  grid on; axis([-8 2 -4 4]); title('Nyquist of G(s)');
subplot(1,2,2); nyquist(Gd); grid on; axis([-8 2 -4 4]); title('Nyquist of G(s) with 90 ms delay');
fprintf('Open loop RHP poles P = %d. Stable closed loop needs N = P counter clockwise loops of -1.\n', ...
        sum(real(pole(G)) > 0));

%% Task 6.2) Bode plot, gain and phase margin
figure('Name','Task 6.2 margins');
subplot(1,2,1); margin(G, {0.1, 50});  grid on;
subplot(1,2,2); margin(G*exp(-Td*s), {0.1, 50}); grid on; title('With 90 ms delay');
disp('All margins, no delay:');        allmargin(G)
disp('All margins, with 90 ms delay:'); allmargin(G*exp(-Td*s))

%% Task 6.3) Design criteria on the uncompensated loop
T0 = feedback(G, 1);
figure('Name','Task 6.3');
step(T0, feedback(Gd,1), feedback(K_nom/s^2,1), 15); grid on;
legend('identified G','G with delay','Part 1 model'); title('Task 6.3: unity feedback, no compensator');
S0 = stepinfo(T0, 'SettlingTimeThreshold', 0.05);
fprintf('No compensator: overshoot %.1f %%, settling (5 %%) %.2f s, final value %.3f\n', ...
        S0.Overshoot, S0.SettlingTime, dcgain(T0));
OS = 5;  Tset = 5;
zeta  = -log(OS/100)/sqrt(pi^2 + log(OS/100)^2);
PMreq = atand(2*zeta/sqrt(sqrt(1+4*zeta^4) - 2*zeta^2));
fprintf('Specs need zeta >= %.2f, PM >= %.1f deg, zeta*wn >= %.2f rad/s\n', zeta, PMreq, 3/Tset);

%% Task 7) Lead compensator  C(s) = Kc (T s + 1)/(alpha T s + 1)
wc    = 4.0;                                    % new gain crossover [rad/s]
alpha = 0.1;                                    % up to 55 deg of phase lead
T     = 1/(wc*sqrt(alpha));                     % peak lead placed at wc
C0    = (T*s + 1)/(alpha*T*s + 1);
Kc    = 1/abs(freqresp(C0*G, wc));              % |C G| = 1 at wc
C     = Kc*C0;
F     = 1/(T*s + 1);                            % prefilter: cancels the lead zero for the reference
fprintf('\nLead: Kc = %.3f deg/cm, T = %.3f s, alpha*T = %.4f s, phi_max = %.1f deg\n', ...
        Kc, T, alpha*T, asind((1-alpha)/(1+alpha)));
L = C*G;
[~, PmL, ~, WcL]   = margin(L);
[~, PmLd]          = margin(L*exp(-Td*s));
fprintf('Compensated PM = %.1f deg at %.2f rad/s (%.1f deg with 90 ms delay)\n', PmL, WcL, PmLd);

figure('Name','Task 7 Bode');
bode(G, L, L*exp(-Td*s), {0.1 30}); grid on;
legend('G (no compensator)','C G','C G with delay'); title('Task 7: Bode before and after the lead');

Tlead  = feedback(L, 1);
TleadF = F*feedback(L*pade(exp(-Td*s),3), 1);
Tnom   = F*feedback(C*K_nom/s^2*pade(exp(-Td*s),3), 1);
figure('Name','Task 7 step');
step(T0, Tlead, TleadF, Tnom, 8); grid on;
legend('no compensator','lead only','lead + prefilter, with delay','lead + prefilter, Part 1 model');
title('Task 7: step responses');
for c = {T0, Tlead, TleadF, Tnom}
    Si = stepinfo(c{1}, 'SettlingTimeThreshold', 0.05);
    fprintf('OS = %5.2f %%   Ts(5%%) = %5.2f s   final value = %.3f\n', Si.Overshoot, Si.SettlingTime, dcgain(c{1}));
end
figure('Name','Task 7 root locus'); rlocus(L); grid on; axis([-15 3 -10 10]); title('Task 7: root locus of C(s)G(s)');

%% For Task 8) Arduino version of the lead and the prefilter (40 ms loop, Tustin)
Cd = c2d(C, 0.04, 'tustin');  Fd = c2d(F, 0.04, 'tustin');
[nC, dC] = tfdata(Cd, 'v');   [nF, dF] = tfdata(Fd, 'v');
fprintf('\nu[k]  = %.4f e[k] + (%.4f) e[k-1] - (%.4f) u[k-1]\n', nC(1), nC(2), dC(2));
fprintf('rf[k] = %.5f r[k] + %.5f r[k-1] - (%.5f) rf[k-1]\n', nF(1), nF(2), dF(2));

%% ------------------------------------------------------------------
function P = chirpPoints(M, servoC, tCut, f0, f1, Tch)
% Gain and phase at the chirp frequency, found by fitting a sine to 1.5 cycles at a time
    t = M(:,1)/1000;  u = M(:,2) - servoC;  y = M(:,3);
    ok = (t >= tCut) & (y > 0);  t = t(ok); u = u(ok); y = y(ok);
    med = movmedian(y,7); bad = abs(y-med) > 2.5; y(bad) = med(bad);
    k = (f1 - f0)/Tch;  phi = 2*pi*(f0*t + 0.5*k*t.^2);
    P = [];  tc = tCut;
    while true
        fi = f0 + k*tc;  win = 1.5/fi;
        m = t >= tc & t < tc + win;
        if sum(m) < 15 || tc + win > t(end), break; end
        X  = [sin(phi(m)) cos(phi(m)) ones(sum(m),1) t(m)-tc];
        cu = X\u(m);  cy = X\y(m);
        Gc = (cy(1) + 1i*cy(2))/(cu(1) + 1i*cu(2));
        P(end+1,:) = [2*pi*(f0 + k*(tc + win/2)), abs(Gc), rad2deg(angle(Gc))]; %#ok<AGROW>
        tc = tc + win/2;
    end
end
