%% ES 245 Part 2: record the chirp test into chirp_data.csv
% Close the Arduino IDE Serial Monitor first (only one program can use the port).
% Opening the port restarts the Arduino. You then have 3 s to put the car
% at the centre before the chirp starts.

clear; clc;
port  = "";                 % "" = pick automatically, or e.g. "COM5" / "/dev/cu.usbmodem1101"
fname = "chirp_data.csv";   % change to chirp_run2.csv etc. for repeat runs

if port == ""
    ports = serialportlist("available");
    disp("Available ports:"); disp(ports');
    port = ports(end);
end
fprintf('Using port %s\n', port);
sp = serialport(port, 115200);
configureTerminator(sp, "CR/LF");
sp.Timeout = 10;
disp('Arduino restarting. Put the car at the centre now (3 s)...');

fid = fopen(fname, 'w');
n = 0;
while true
    line = strtrim(readline(sp));
    if strlength(line) == 0, continue; end
    if line == "END", break; end
    fprintf(fid, '%s\n', line);
    n = n + 1;
    if mod(n, 50) == 0, fprintf('%d lines, last: %s\n', n, line); end
end
fclose(fid);
clear sp;
fprintf('Saved %d lines to %s\n', n, fname);

% Quick look
M = readmatrix(fname);
M = M(M(:,3) > 0, :);
figure;
subplot(2,1,1); plot(M(:,1)/1000, M(:,2)); grid on; ylabel('servo [deg]');
subplot(2,1,2); plot(M(:,1)/1000, M(:,3)); grid on; ylabel('distance [cm]'); xlabel('t [s]');
