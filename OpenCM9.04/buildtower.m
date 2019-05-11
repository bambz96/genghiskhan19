% successfully sends polynomial coefficients
clear all % remove serial that insists on hanging around and fucking shit up
close all
serial = serial('COM4','BAUD',57600);

%% 4 polynomial coeffs

tf = 3;
tf2 = 3;
tf3 = 3;
tf4 = 3; 

% X     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a3, a2, a1, a0] = cubic_coeffs(0.25, 0, 0.037, 0, tf);
xdata = [a3 a2 a1 a0 tf];
[a3, a2, a1, a0] = cubic_coeffs(0.037, 0, 0.25, 0, tf2);
xdata = [xdata; a3 a2 a1 a0 tf2];
[a3, a2, a1, a0] = cubic_coeffs(0.25, 0, 0.037, 0, tf3);
xdata = [xdata; a3 a2 a1 a0 tf3];
[a3, a2, a1, a0] = cubic_coeffs(0.037, 0, 0.25, 0, tf4);
xdata = [xdata;a3 a2 a1 a0 tf4];
% Y     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a3, a2, a1, a0] = cubic_coeffs(0, 0, -0.1875, 0, tf);
ydata = [a3 a2 a1 a0 tf];
[a3, a2, a1, a0] = cubic_coeffs(-0.1875, 0, 0, 0, tf2);
ydata = [ydata; a3 a2 a1 a0 tf2];
[a3, a2, a1, a0] = cubic_coeffs(0, 0, 0.1875, 0, tf3);
ydata = [ydata; a3 a2 a1 a0 tf3];
[a3, a2, a1, a0] = cubic_coeffs(0.1875, 0, 0, 0, tf4);
ydata = [ydata;a3 a2 a1 a0 tf4];
% Z     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a3, a2, a1, a0] = cubic_coeffs(0.27, 0, 0.015, 0, tf);
zdata = [a3 a2 a1 a0 tf];
[a3, a2, a1, a0] = cubic_coeffs(0.015, 0, 0.27, 0, tf2);
zdata = [zdata; a3 a2 a1 a0 tf2];
[a3, a2, a1, a0] = cubic_coeffs(0.27, 0, 0.015, 0, tf3);
zdata = [zdata; a3 a2 a1 a0 tf3];
[a3, a2, a1, a0] = cubic_coeffs(0.015, 0, 0.27, 0, tf4);
zdata = [zdata; a3 a2 a1 a0 tf4];
% THETA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a3, a2, a1, a0] = cubic_coeffs(0, 0, 1.55, 0, tf);
thdata = [a3 a2 a1 a0 tf];
[a3, a2, a1, a0] = cubic_coeffs(1.55, 0, 0, 0, tf2);
thdata = [thdata; a3 a2 a1 a0 tf2];
[a3, a2, a1, a0] = cubic_coeffs(0, 0, -1.55, 0, tf3);
thdata = [thdata; a3 a2 a1 a0 tf3];
[a3, a2, a1, a0] = cubic_coeffs(-1.55, 0, 0, 0, tf4);
thdata = [thdata; a3 a2 a1 a0 tf4];

% number of rows
length = 4;

% length = 2;
% tv = 0.2; tf = 0.4;
% [a3, a2, a1, a0, b3, b2, b1, b0] = cubic_via_coeffs(0, 0, 1, 0, 0.3, tv, tf);
% xdata = [a3 a2 a1 a0 tv; b3 b2 b1 b0 tf-tv];
% [a3, a2, a1, a0, b3, b2, b1, b0] = cubic_via_coeffs(1, 0, 0, 0, 0.3, tv, tf);
% ydata = [a3 a2 a1 a0 tv; b3 b2 b1 b0 tf-tv];
% [a3, a2, a1, a0, b3, b2, b1, b0] = cubic_via_coeffs(0.5, 0, 0.5, 0, 1, tv, tf);
% zdata = [a3 a2 a1 a0 tv; b3 b2 b1 b0 tf-tv];
% [a3, a2, a1, a0, b3, b2, b1, b0] = cubic_via_coeffs(0.2, 0, .2, 0, 0, tv, tf);
% thdata = [a3 a2 a1 a0 tv; b3 b2 b1 b0 tf-tv];

%% open serial and wait to establish
fopen(serial);
pause(1.5);

%% send number of rows about to be sent
tic
fprintf(serial, string(length));
reply = strtrim(fscanf(serial));
if ~strcmp(join(string(reply)), string(length))
    disp('Device did not agree on length, will not send. '+join(string(reply)));
    fclose(serial);
    delete(serial);
    return;
else
    disp('Device agreed on length of '+string(length)+'.')
end
toc

%% send rows
tic
sendRow(serial, xdata);
sendRow(serial, ydata);
sendRow(serial, zdata);
sendRow(serial, thdata);
toc

fclose(serial);
delete(serial);
return

%% received plotting data
[tx, x] = readRow(serial, length);
disp('Received x trajectory.')
[ty, y] = readRow(serial, length);
disp('Received y trajectory.')
[tz, z] = readRow(serial, length);
disp('Received z trajectory.')
[tth, th] = readRow(serial, length);
disp('Received theta trajectory.')
subplot(121)
hold on
plot(tx, x)
plot(ty, y)
plot(tz, z)
plot(tth, th)
legend('x', 'y', 'z', 'theta')
xlabel('Time (s)')
subplot(122)
plot3(x,y,z)
xlabel('x')
ylabel('y')
zlabel('z')
%% clean up
fclose(serial);
delete(serial);

function sendRow(serial, data)
    length = size(data);
    for i = 1:length
        send = join(string(data(i, :)));
        disp("send: " + send);
        fprintf(serial, send);
        reply = strtrim(fscanf(serial));
        % if ~strcmp(string(i), reply)
        %     disp(reply);
        %    errors = errors + 1; 
        % end
        disp("recv: " + reply);
    end
end

function [t, d] = readRow(serial, length)
    i = 1;
    t = zeros(1, 100*length - 1);
    d = zeros(1, 100*length - 1);
    while i <= 100*length
        data = strtrim(fscanf(serial));
        res = regexp(data, '[+-]?\d+\.?\d*','match');
        t(i) = str2double(res{1});
        d(i) = str2double(res{2});
        i = i + 1;
    end
end