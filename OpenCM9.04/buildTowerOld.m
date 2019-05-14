% successfully sends polynomial coefficients
clear all % remove serial that insists on hanging around and fucking shit up
close all
serial = serial('COM4','BAUD',57600);

%% 4 polynomial coeffs

tf = 2;
tf2 = 1;
tf3 = 1;
tf4 = 1;
tf5 = 1; 
tf6 = 2; 

grip = 1.015976119; 
ungrip = 0.422257077; 


% X     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a3, a2, a1, a0] = cubic_coeffs(0.2, 0, 0.15, 0, tf);
xdata = [a3 a2 a1 a0 tf];
[a3, a2, a1, a0] = cubic_coeffs(0.15, 0, 0.2, 0, tf2);
xdata = [xdata; a3 a2 a1 a0 tf2];
[a3, a2, a1, a0] = cubic_coeffs(0.2, 0, 0.125, 0, tf3);
xdata = [xdata; a3 a2 a1 a0 tf3];
[a3, a2, a1, a0] = cubic_coeffs(0.125, 0, 0.2, 0, tf4);
xdata = [xdata;a3 a2 a1 a0 tf4];
[a3, a2, a1, a0] = cubic_coeffs(0.2, 0, 0.15, 0, tf5);
xdata = [xdata; a3 a2 a1 a0 tf5];
[a3, a2, a1, a0] = cubic_coeffs(0.15, 0, 0.2, 0, tf6);
xdata = [xdata;a3 a2 a1 a0 tf6];
% Y     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a3, a2, a1, a0] = cubic_coeffs(0, 0, -0.2, 0, tf);
ydata = [a3 a2 a1 a0 tf];
[a3, a2, a1, a0] = cubic_coeffs(-0.2, 0, 0, 0, tf2);
ydata = [ydata; a3 a2 a1 a0 tf2];
[a3, a2, a1, a0] = cubic_coeffs(0, 0, 0.2, 0, tf3);
ydata = [ydata; a3 a2 a1 a0 tf3];
[a3, a2, a1, a0] = cubic_coeffs(0.2, 0, 0, 0, tf4);
ydata = [ydata;a3 a2 a1 a0 tf4];
[a3, a2, a1, a0] = cubic_coeffs(0, 0, -0.2, 0, tf5);
ydata = [ydata; a3 a2 a1 a0 tf5];
[a3, a2, a1, a0] = cubic_coeffs(-0.2, 0, 0, 0, tf6);
ydata = [ydata;a3 a2 a1 a0 tf6];
% Z     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a3, a2, a1, a0] = cubic_coeffs(0.3, 0, 0.01, 0, tf);
zdata = [a3 a2 a1 a0 tf];
[a3, a2, a1, a0] = cubic_coeffs(0.01, 0, 0.01, 0, tf2);
zdata = [zdata; a3 a2 a1 a0 tf2];
[a3, a2, a1, a0] = cubic_coeffs(0.01, 0, 0.01, 0, tf3);
zdata = [zdata; a3 a2 a1 a0 tf3];
[a3, a2, a1, a0] = cubic_coeffs(0.01, 0, 0.01, 0, tf4);
zdata = [zdata; a3 a2 a1 a0 tf4];
[a3, a2, a1, a0] = cubic_coeffs(0.01, 0, 0.01, 0, tf5);
zdata = [zdata; a3 a2 a1 a0 tf5];
[a3, a2, a1, a0] = cubic_coeffs(0.01, 0, 0.3, 0, tf6);
zdata = [zdata; a3 a2 a1 a0 tf6];
% THETA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a3, a2, a1, a0] = cubic_coeffs(0, 0, 0, 0, tf);
thdata = [a3 a2 a1 a0 tf];
[a3, a2, a1, a0] = cubic_coeffs(0, 0, 0, 0, tf2);
thdata = [thdata; a3 a2 a1 a0 tf2];
[a3, a2, a1, a0] = cubic_coeffs(0, 0, 0, 0, tf3);
thdata = [thdata; a3 a2 a1 a0 tf3];
[a3, a2, a1, a0] = cubic_coeffs(0, 0, 0, 0, tf4);
thdata = [thdata; a3 a2 a1 a0 tf4];
[a3, a2, a1, a0] = cubic_coeffs(0, 0, 0, 0, tf5);
thdata = [thdata; a3 a2 a1 a0 tf5];
[a3, a2, a1, a0] = cubic_coeffs(0, 0, 0, 0, tf6);
thdata = [thdata; a3 a2 a1 a0 tf6];
% GRIP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a3, a2, a1, a0] = cubic_coeffs(ungrip, 0, ungrip, 0, tf); % GRIP
gripdata = [a3 a2 a1 a0 tf];
[a3, a2, a1, a0] = cubic_coeffs(ungrip, 0, ungrip, 0, tf2); % UNGRIP
gripdata = [gripdata; a3 a2 a1 a0 tf2];
[a3, a2, a1, a0] = cubic_coeffs(ungrip, 0, ungrip, 0, tf3);
gripdata = [gripdata; a3 a2 a1 a0 tf3];
[a3, a2, a1, a0] = cubic_coeffs(ungrip, 0, ungrip, 0, tf4);
gripdata = [gripdata; a3 a2 a1 a0 tf4];
[a3, a2, a1, a0] = cubic_coeffs(ungrip, 0, ungrip, 0, tf5);
gripdata = [gripdata; a3 a2 a1 a0 tf5];
[a3, a2, a1, a0] = cubic_coeffs(ungrip, 0, ungrip, 0, tf6);
gripdata = [gripdata; a3 a2 a1 a0 tf6];

% number of rows
length = 6;

% %%%%%%%%%%% GRIP TEST %%%%%%%%%%%
% [a3, a2, a1, a0] = cubic_coeffs(0.2, 0, 0.2, 0, tf);
% xdata = [a3 a2 a1 a0 tf];
% [a3, a2, a1, a0] = cubic_coeffs(0.2, 0, 0.2, 0, tf2);
% xdata = [xdata; a3 a2 a1 a0 tf2];
% [a3, a2, a1, a0] = cubic_coeffs(0, 0, 0, 0, tf);
% ydata = [a3 a2 a1 a0 tf];
% [a3, a2, a1, a0] = cubic_coeffs(0, 0, 0, 0, tf2);
% ydata = [ydata; a3 a2 a1 a0 tf2];
% [a3, a2, a1, a0] = cubic_coeffs(0.3, 0, 0.3, 0, tf);
% zdata = [a3 a2 a1 a0 tf];
% [a3, a2, a1, a0] = cubic_coeffs(0.3, 0, 0.3, 0, tf2);
% zdata = [zdata; a3 a2 a1 a0 tf2];
% [a3, a2, a1, a0] = cubic_coeffs(0, 0, 0, 0, tf);
% thdata = [a3 a2 a1 a0 tf];
% [a3, a2, a1, a0] = cubic_coeffs(0, 0, 0, 0, tf2);
% thdata = [thdata; a3 a2 a1 a0 tf2];
% [a3, a2, a1, a0] = cubic_coeffs(0.422257077, 0, 1.015976119, 0, tf); % GRIP
% gripdata = [a3 a2 a1 a0 tf];
% [a3, a2, a1, a0] = cubic_coeffs(1.015976119, 0, 0.422257077, 0, tf2); % UNGRIP
% gripdata = [gripdata; a3 a2 a1 a0 tf2];
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
sendRow(serial, gripdata);
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
[tgrip, grip] = readRow(serial, length);
disp('Received grip trajectory.')
subplot(121)
hold on
plot(tx, x)
plot(ty, y)
plot(tz, z)
plot(tth, th)
plot(tgrip,grip)
legend('x', 'y', 'z', 'theta','grip')
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