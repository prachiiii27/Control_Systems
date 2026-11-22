%% ES 245 Task 4: record one hardware test from the Arduino into a CSV file
% Close the Arduino IDE Serial Monitor / Plotter first: only one program
% can use the USB port at a time.
%
% Opening the port restarts the Arduino, so this script sends your
% calibration values again before every test.

clear; clc;

%% 1) Settings: edit these before each run
port     = "";           % "" = pick automatically. Or e.g. "/dev/cu.usbmodem1101" (Mac), "COM5" (Windows)
level    = 90.0;         % SERVO_LEVEL you found in calibration (deg)
dirSign  = 1;            % DIR you found in calibration (+1 or -1)
centre   = 18.75;        % sensor reading with the car over the pivot (cm)
modeNum  = 2;            % 1 = P, 2 = PD, 3 = PID
duration = 45;           % seconds to record (four 10 s steps plus a margin)
names    = ["", "P", "PD", "PID"];
fname    = "log_" + names(modeNum + 1) + ".csv";   % e.g. log_PD.csv

%% 2) Open the port
if port == ""
    ports = serialportlist("available");
    disp("Available ports:"); disp(ports');
    port = ports(end);
end
fprintf('Using port %s\n', port);
sp = serialport(port, 115200);
configureTerminator(sp, "LF");
sp.Timeout = 2;
pause(2.5);            % wait for the Arduino to finish restarting
flush(sp);

%% 3) Send calibration and start the step test
cmds = ["l" + level, "c" + centre, "d" + dirSign, "m" + modeNum, "s1", "r", "o1"];
for c = cmds
    writeline(sp, c);
    pause(0.15);
end

%% 4) Record
fid = fopen(fname, 'w');
started = false;
t0 = tic;
fprintf('Recording %s for %d s. Do not touch the beam.\n', fname, duration);
while toc(t0) < duration
    line = readline(sp);
    if isempty(line), continue; end
    line = strtrim(line);
    if startsWith(line, "# t_s"), started = true; end   % CSV header marks the start
    if started
        fprintf(fid, '%s\n', line);
    end
end
fclose(fid);

%% 5) Stop the step test and close the port
writeline(sp, "s0");
writeline(sp, "o0");
clear sp
fprintf('Saved %s\n', fname);
