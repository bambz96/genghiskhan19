clear serial % remove any serial that is hanging around and fucking shit up
close all
serial = serial('COM3','BAUD',57600);

running = 1;
while running
    disp('1 - send build tower trajectories now')
%     disp('2 - send trajectory from to position from current')
%     disp('3 - read current EE')
%     disp('4 - read current joint angles')
%     disp('5 - 1 and plot after')
%     disp('6 - 2 and plot after')
    disp('0 - quit')

    user = input('>');

    if user == 1
        disp('Send build tower trajectories now')
        [length, xdata, ydata, zdata, thdata, gripdata] = tower_paths();
        %% pause to make sure it's opened
        fopen(serial);
        pause(1.5);
        %% send number of rows about to be sent
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
        %% send rows
        tic
        disp('Send x')
        sendRow(serial, xdata);
        disp('Send y')
        sendRow(serial, ydata);
        disp('Send z')
        sendRow(serial, zdata);
        disp('Send theta')
        sendRow(serial, thdata);
        disp('Send grip')
        sendRow(serial, gripdata);
        toc
%         %% receive plotting data
%         close all
%         [tx, x] = readRow(serial, length);
%         disp('Received x trajectory.')
%         [ty, y] = readRow(serial, length);
%         disp('Received y trajectory.')
%         [tz, z] = readRow(serial, length);
%         disp('Received z trajectory.')
%         [tth, th] = readRow(serial, length);
%         disp('Received theta trajectory.')
%         [tgrip, grip] = readRow(serial, length);
%         disp('Received grip trajectory.')
%         subplot(121)
%         hold on
%         plot(tx, x)
%         plot(ty, y)
%         plot(tz, z)
%         plot(tth, th)
%         plot(tgrip,grip)
%         legend('x', 'y', 'z', 'theta','grip')
%         xlabel('Time (s)')
%         subplot(122)
%         plot3(x,y,z)
%         xlabel('x')
%         ylabel('y')
%         zlabel('z')
        %% close
        fclose(serial);
    elseif user == 2
        disp('Send trajectory from to position from current')

    elseif user == 3
        disp('Read current EE')

    elseif user == 4
        disp('Read current joint angles')

    elseif user == 0
        disp('Quitting')
        fclose(serial);
        delete(serial);
        clear serial
        running = 0;
    end
end
return

function sendRow(serial, data)
    [length,~] = size(data);
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
    disp('Waiting for data...')
    i = 1;
    t = zeros(1, 100*length);
    d = zeros(1, 100*length);
    while i <= 100*length
        data = strtrim(fscanf(serial));
        res = regexp(data, '[+-]?\d+\.?\d*','match');
        t(i) = str2double(res{1});
        d(i) = str2double(res{2});
        i = i + 1;
    end
end

function [length, xdata, ydata, zdata, thdata, gripdata] = tower_paths()

grip = 1.015976119; 
ungrip = 0.422257077;

% metres, degrees, degrees, ?, seconds
% x y z theta grip duration
% p = [
%     0 1 0.2 0 grip 1;
%     1 0 0.5 90 ungrip 1;
%     0 1 0.1 0 grip 1;
% ];
% tower corner
x0 = 0.2;
y0 = -0.2;
% home pos
xh = 0.0375;
yh = -0.1875;
zh = 0.003; % -0.003
thetah = -90;

p = [];
nblocks = 3;
i = 1; % number of loops
n = 1; % up to block
while n <= nblocks
    if mod(i,2) == 0 % i = 2, 4, 6...
        [x, y, z, theta] = block_pos(n,x0,y0);
        n = n + 1;
        % move to above target holding block
        p = [p; x y z+0.05 theta grip 2];
        % lower
        p = [p; x y z+0.005 theta grip 1];
        % stay at point and release block
        p = [p; x y z+0.005 theta ungrip 0.3];
        % raise again
        p = [p; x y z+0.005 theta ungrip 1];
    else
        % move to above block pick up
        p = [p; xh yh zh+0.05 thetah ungrip 2];
        % move down
        p = [p; xh yh zh thetah ungrip 2];
        % nudge forwards to fit block precisely and pick up
        p = [p; xh+0.001 yh zh thetah grip 0.3];
        % move up
        p = [p; xh yh zh+0.05 thetah grip 1];
    end
    i = i + 1;
end

disp(p)

[length, ~] = size(p);
length = length - 1;

xdata = zeros(length, 5);
ydata = zeros(length, 5);
zdata = zeros(length, 5);
thdata = zeros(length, 5);
gripdata = zeros(length, 5);

%  current to p(1), the first pose, in 3 seconds

for i = 1:length
    x0 = p(i,1); xf = p(i+1,1);
    y0 = p(i,2); yf = p(i+1,2);
    z0 = p(i,3); zf = p(i+1,3);
    th0 = p(i,4)*pi/180; thf = p(i+1,4)*pi/180;
    gr0 = p(i,5); grf = p(i+1,5);
    tf = p(i,6);
    xdata(i,:) = cubic_coeffs(x0, 0, xf, 0, tf);
    ydata(i,:) = cubic_coeffs(y0, 0, yf, 0, tf);
    zdata(i,:) = cubic_coeffs(z0, 0, zf, 0, tf);
    thdata(i,:) = cubic_coeffs(th0, 0, thf, 0, tf);
    gripdata(i,:) = cubic_coeffs(gr0, 0, grf, 0, tf);
end


end