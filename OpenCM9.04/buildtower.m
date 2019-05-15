clear serial % remove any serial that is hanging around and fucking shit up
close all
serial = serial('COM4','BAUD',57600);

openTime = 1; % seconds paused after serial opened

%% pause to make sure it's opened
fopen(serial);
pause(openTime);

running = 1;
while running
    disp('1 - send trajectory to position from current')
    disp('2 - send test trajectories')
    disp('3 - plot all trajectories stored on device')
    disp('5 - run position control on stored trajectories')
    disp('6 - run velocity control on stored trajectories')
    disp('7 - passively read joints and EE position')
    disp('0 - quit')

    user = input('>');

    if user == 1
        disp('Send trajectory to position from current...')
        disp('Not implemented!')
    elseif user == 2
        disp('Send build tower trajectories now...')
        [length, xdata, ydata, zdata, thdata, gripdata] = tower_paths();
        %% send command
        fprintf(serial, 'N');
        reply = strtrim(fscanf(serial));
        if ~strcmp(join(string(reply)), 'N')
            disp("Device did not reply correctly, expected 'N', got: "+join(string(reply)));
            fclose(serial);
            delete(serial);
            return;
        else
            disp('Device agreed to receive trajectories.')
        end
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
    elseif user == 3
        disp('Plot all trajectories stored on device...')
        %% send command, device will respond with path res
        fprintf(serial, 'P');
        n_path = str2num(strtrim(fgetl(serial)));
        if n_path == 0
            disp('Device has no stored paths.')
            disp('-------------------------------------')
            continue
        end
        disp('Device is sending ' + string(n_path) + ' paths.')
        path_res = str2num(strtrim(fgetl(serial)));
        disp('Device is sending paths with resolution: '+string(path_res)+'.')
        %% receive plotting data
        close all
        [tx, x] = readRow(serial, n_path, path_res);
        disp('Received x trajectory.')
        [ty, y] = readRow(serial, n_path, path_res);
        disp('Received y trajectory.')
        [tz, z] = readRow(serial, n_path, path_res);
        disp('Received z trajectory.')
        [tth, th] = readRow(serial, n_path, path_res);
        disp('Received theta trajectory.')
        [tgrip, grip] = readRow(serial, n_path, path_res);
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
    elseif user == 5
        disp('Run position control on stored trajectories...')
        fprintf(serial, "PC");
        received = strtrim(fscanf(serial));
        if strcmp(received, "PC")
            disp('Beginning position control.')
        else
            disp('Device did not respond correctly: ' + received)
        end
    elseif user == 6
        disp('Run velocity control on stored trajectories...')
        fprintf(serial, "VC");
        received = strtrim(fscanf(serial));
        if strcmp(received, "VC")
            disp('Beginning velocity control.')
        else
            disp('Device did not respond correctly: ' + received)
        end
    elseif user == 7
        disp('Passively read joints and EE position...')
        fprintf(serial, 'R');
        i = 0;
        while i < 500
            disp(fgetl(serial));
            i = i + 1;
        end
        fprintf(serial, 'x'); % send anything to interrupt
        
        % todo with GUI, will be able to send anything over serial and
        % return to WAITING state, atm have to use Arduino's Serial Monitor
        
        % simple GUI used to interrupt loop and view joint angles
%         ButtonHandle = uicontrol('Style', 'PushButton', 'String', 'Stop read', 'Callback', 'delete(gcbf)');
%         textHandle = uicontrol('Style', 'text', 'String', 'q1 q2 q3 q4 q5', 'Position', [150, 150, 300, 15]);
%         while 1
%             if ~ishandle(ButtonHandle)
%                 disp('Ended passive reading.');
%                 break;
%             end
%             textHandle.String = fgetl(serial);
%         end
        
    elseif user == 0
        disp('Quitting')
        fclose(serial);
        delete(serial);
        clear serial
        running = 0;
    end
    disp('-------------------------------------')
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

function [t, d] = readRow(serial, length, path_res)
    disp('Waiting for data...')
    i = 1;
    t = zeros(1, path_res*length);
    d = zeros(1, path_res*length);
    while i <= path_res*length
        data = strtrim(fscanf(serial));
        disp(data)
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
delay = 5;
p = [
    0.2 0 0.3 0 ungrip 2;
    0.15 -0.2 0.01 0 ungrip 7;
    0.15 -0.2 0.01 0 ungrip 8;
    0.2 0 0.01 0 ungrip 13;
    0.2 0 0.01 0 ungrip 14;
    0.125 0.25 0.01 0 ungrip 19;
    0.125 0.25 0.01 0 ungrip 20;
    0.2 0 0.01 0 ungrip 25;
    0.2 0 0.01 0 ungrip 26;
    0.15 -0.2 0.01 0 ungrip 31;
    0.15 -0.2 0.01 0 ungrip 32;
    0.2 0 0.01 0 ungrip 37;
    0.2 0 0.01 0 ungrip 42;
    0.2 0 0.3 0 ungrip 44
];
% tower corner
% x0 = 0.2;
% y0 = -0.2;
% % home pos
% xh = 0.0375;
% yh = -0.1875;
% zh = 0.003; % -0.003
% thetah = -90;
% 
% p = [];
% nblocks = 3;
% i = 1; % number of loops
% n = 1; % up to block
% while n <= nblocks
%     if mod(i,2) == 0 % i = 2, 4, 6...
%         [x, y, z, theta] = block_pos(n,x0,y0);
%         n = n + 1;
%         % move to above target holding block
%         p = [p; x y z+0.05 theta grip 2];
%         % lower
%         p = [p; x y z+0.005 theta grip 1];
%         % stay at point and release block
%         p = [p; x y z+0.005 theta ungrip 0.3];
%         % raise again
%         p = [p; x y z+0.005 theta ungrip 1];
%     else
%         % move to above block pick up
%         p = [p; xh yh zh+0.05 thetah ungrip 2];
%         % move down
%         p = [p; xh yh zh thetah ungrip 2];
%         % nudge forwards to fit block precisely and pick up
%         p = [p; xh+0.001 yh zh thetah grip 0.3];
%         % move up
%         p = [p; xh yh zh+0.05 thetah grip 1];
%     end
%     i = i + 1;
% end
% p = [p; xh yh zh+0.05 thetah ungrip 2];

disp(p)

[length, ~] = size(p);
length = length - 1;

xdata = zeros(length, 6);
ydata = zeros(length, 6);
zdata = zeros(length, 6);
thdata = zeros(length, 6);
gripdata = zeros(length, 6);

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
