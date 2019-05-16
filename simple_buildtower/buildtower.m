clear serial % remove any serial that is hanging around and fucking shit up
close all
%% select serial port
disp('Available COM ports:')
disp(join(seriallist))
i = input('COM number:');
serial = serial('COM'+string(i),'BAUD',57600);
%% pause to make sure it's opened
fopen(serial);
pause(1);
disp('Connected to COM'+string(i)+'.')
%% main loop
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
        sendTrajectories(serial, DATA);
    elseif user == 3
        plotStoredTrajectories(serial);
    elseif user == 5
        runPositionControl(serial);
    elseif user == 6
        runVelocityControl(serial);
    elseif user == 7
        readJoints(serial);
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
%% functions
function sendTrajectories(serial, data)
    disp('Send build tower trajectories now...')
    % [length, xdata, ydata, zdata, thdata, gripdata] = tower_paths();
    xdata = data(:,:,1);
    ydata = data(:,:,2);
    zdata = data(:,:,3);
    thdata = data(:,:,4);
    gripdata = data(:,:,5);
    [length, ~, ~] = size(data);
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
end

function plotStoredTrajectories(serial)
    disp('Plot all trajectories stored on device...')
    %% send command, device will respond with path res
    fprintf(serial, 'P');
    n_path = str2num(strtrim(fgetl(serial)));
    if n_path == 0
        disp('Device has no stored paths.')
        disp('-------------------------------------')
        return
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
end

function runPositionControl(serial)
    disp('Run position control on stored trajectories...')
    fprintf(serial, "PC");
    received = strtrim(fscanf(serial));
    if strcmp(received, "PC")
        disp('Beginning position control.')
    else
        disp('Device did not respond correctly: ' + received)
    end
end

function runVelocityControl(serial)
    disp('Run velocity control on stored trajectories...')
    fprintf(serial, "VC");
    received = strtrim(fscanf(serial));
    if strcmp(received, "VC")
        disp('Beginning velocity control.')
    else
        disp('Device did not respond correctly: ' + received)
    end
end

function readJoints(serial)
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
%     ButtonHandle = uicontrol('Style', 'PushButton', 'String', 'Stop read', 'Callback', 'delete(gcbf)');
%     textHandle = uicontrol('Style', 'text', 'String', 'q1 q2 q3 q4 q5', 'Position', [150, 150, 300, 15]);
%     while 1
%         if ~ishandle(ButtonHandle)
%             disp('Ended passive reading.');
%             break;
%         end
%         textHandle.String = fgetl(serial);
%     end
end

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
        res = regexp(data, '[+-]?\d+\.?\d*','match');
        t(i) = str2double(res{1});
        d(i) = str2double(res{2});
        i = i + 1;
    end
end

