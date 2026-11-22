%% ES 245 Course Project - Part 1 : Tasks 2 and 3 (MATLAB side)
% Car-on-beam with servo four-bar linkage, Setup 5
% Plant (linearised):  G(s) = A(s)/Theta(s) = K / s^2 ,  K = g*d/L
%
% RUN THIS SCRIPT BEFORE OPENING / RUNNING THE SIMULINK MODEL.
% It puts every parameter and gain in the MATLAB workspace, and the
% Simulink blocks read them by name (K, Kp_PD, Kd_PD, ...).
%
% Units used everywhere in MATLAB/Simulink:
%   car position a  -> metres (m)
%   servo angle th  -> radians (rad)

clear; clc; close all;

%% 1) Measured parameters (Setup 5)
g      = 9.81;       % gravity                                   [m/s^2]
m      = 0.03634;    % car mass (36.34 g)                         [kg]
d      = 0.05;       % servo horn: shaft centre -> horn pin       [m]
l_arm  = 0.11;       % lever arm (coupler link) length            [m]
L      = 0.25;       % beam pivot -> lever-arm pin on the beam    [m]

% Track / sensor geometry (all measured along the beam)
track_start = 0.055;           % track starts 5.5 cm from lever pin   [m]
track_len   = 0.39;            % track length                          [m]
car_len     = 0.075;           % car length                            [m]
sensor_gap  = 0.03;            % sensor is 3 cm behind track start     [m]

a_min = sensor_gap;                          % car touching sensor end   (0.030 m)
a_max = sensor_gap + track_len - car_len;    % car touching far end      (0.345 m)
fprintf('Usable sensor reading range: %.1f cm to %.1f cm\n', a_min*100, a_max*100);

% Reading when the car's CENTRE is right above the beam pivot
% (pivot is L = 25 cm from the lever pin; track centre is also at 25 cm)
a_centre = (L - track_start) - car_len/2 + sensor_gap;
fprintf('Sensor reading with car centred over pivot: %.2f cm\n', a_centre*100);

%% 2) Plant
% Toy car: wheel inertia J is very small, so J/R^2 << m and the mass cancels.
K = g*d/L;                                   % = 1.962 m/(rad s^2)
fprintf('Plant gain K = %.3f m/(rad*s^2)\n', K);

s = tf('s');
G = K/s^2;
disp('Open-loop transfer function:'); G

% State-space form (Task 1c), states x1 = a, x2 = a_dot
A  = [0 1; 0 0];
B  = [0; K];
C  = [1 0];
D  = 0;
sys_ss = ss(A,B,C,D);

%% Task 2(a) : Pole-zero map
figure('Name','Task 2a');
pzmap(G); grid on;
title('Task 2(a): Pole-zero map of G(s) = 1.962/s^2');
disp('Poles:'); disp(pole(G));
disp('Zeros:'); disp(zero(G));

%% Task 2(b) : Open-loop step response
% A 1 rad servo step is huge, so use a small, realistic 2 degree step.
th_step = deg2rad(2);
figure('Name','Task 2b');
step(th_step*G, 3); grid on;
title('Task 2(b): Open-loop response to a 2^\circ servo step');
ylabel('Car position change [m]');

%% Task 2(c) : System type
nOrigin = sum(abs(pole(G)) < 1e-9);
fprintf('Poles at origin = %d  ->  Type-%d system\n', nOrigin, nOrigin);

%% Task 3 : Controller gains (SI units: rad per metre)
% Derivative acts on the measured position (c = 0), so a set-point jump
% does not produce a violent servo kick. PID also uses set-point weight b.
N      = 100;                 % derivative filter coefficient (Simulink default)
th_max = deg2rad(30);         % servo limit about the level position

Kp_P  = 1.15;                                     % P only
Kp_PD = 1.15;  Kd_PD = 1.20;                      % PD   (b = 1, c = 0)
Kp    = 2.40;  Ki = 0.80;  Kd = 1.70;  b = 0.7;   % PID  (b = 0.7, c = 0)

%% Task 3 : Closed-loop responses (2-DOF form: u = Cr*r - Cy*y)
Tf = 1/N;
% P
T_P   = feedback(Kp_P*G, 1);
% PD : Cr = Kp,   Cy = Kp + Kd s/(Tf s + 1)
Cr_PD = Kp_PD;
Cy_PD = Kp_PD + Kd_PD*s/(Tf*s+1);
T_PD  = minreal(Cr_PD*feedback(G, Cy_PD));
% PID: Cr = b Kp + Ki/s,   Cy = Kp + Ki/s + Kd s/(Tf s + 1)
Cr_PID = b*Kp + Ki/s;
Cy_PID = Kp + Ki/s + Kd*s/(Tf*s+1);
T_PID  = minreal(Cr_PID*feedback(G, Cy_PID));

figure('Name','Task 3');
step(T_P, T_PD, T_PID, 10); grid on;
legend('P','PD','PID','Location','southeast');
title('Task 3: Closed-loop unit step responses');

disp('---- PD ----');  disp(stepinfo(T_PD,  'SettlingTimeThreshold',0.02));
disp('---- PID ----'); disp(stepinfo(T_PID, 'SettlingTimeThreshold',0.02));
fprintf('P controller closed-loop poles: '); disp(pole(T_P).');

%% Gains converted for the Arduino (cm and degrees)
cf = (180/pi)/100;            % rad/m  ->  deg/cm   (= 0.573)
fprintf('\nArduino gains (deg per cm):\n');
fprintf('  P  : Kp = %.2f\n', Kp_P*cf);
fprintf('  PD : Kp = %.2f  Kd = %.2f\n', Kp_PD*cf, Kd_PD*cf);
fprintf('  PID: Kp = %.2f  Ki = %.2f  Kd = %.2f  b = %.1f\n', Kp*cf, Ki*cf, Kd*cf, b);
