clear serial % remove any serial that is hanging around and fucking shit up
close all
%% init
xsent = 0;
ysent = 0;
zsent = 0;
thsent = 0;
gripsent = 0;
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
    disp('2 - select a trajectory to send')
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
        [xsent, ysent, zsent, thsent, gripsent] = sendTrajectories(serial);
    elseif user == 3
        plotStoredTrajectories(serial, xsent, ysent, zsent, thsent, gripsent);
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
function [xdata, ydata, zdata, thdata, gripdata] = sendTrajectories(serial)
    disp('Select a trajectory to send:')
    disp('1 - test trajectory')
    disp('2 - build tower')
    select = input('>');
    if select == 1
        [length, xdata, ydata, zdata, thdata, gripdata] = create_test_trajectory();
    elseif select == 2
        [length, data] = create_all_tower_paths();
        xdata = data(:,:,1);
        ydata = data(:,:,2);
        zdata = data(:,:,3);
        thdata = data(:,:,4);
        gripdata = data(:,:,5);
    end
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

function plotStoredTrajectories(serial, xsent, ysent, zsent, thsent, gripsent)
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
    %% generate trajectories from sent to validate against received trajectories
    [tx2, x2] = generate(xsent, 1);
    [ty2, y2] = generate(ysent, 1);
    [tz2, z2] = generate(zsent, 1);
    [tth2, th2] = generate(thsent, 1);
    [tgrip2, grip2] = generate(gripsent, 1);
    %% plot
    az = 15;
    el = 15;
    
    subplot(221)
    hold on
    % received
    plot(tx, x)
    plot(ty, y)
    plot(tz, z)
    plot(tth, th)
    plot(tgrip,grip)
    % generated from what was sent
    plot(tx2, x2, 'o')
    plot(ty2, y2, 'o')
    plot(tz2, z2, 'o')
    plot(tth2, th2, 'o')
    plot(tgrip2,grip2, 'o')
    legend('x', 'y', 'z', 'theta','grip')
    xlabel('Time (s)')
    title('Received Trajectories')
    
    subplot(222)
    axis square
    hold on
    grid on
    view(az,el)
    plot3(x,y,z)
    plot3(x2,y2,z2, 'o')
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('Received Trajectories')
    legend('Received', 'Planned')
    
    subplot(223)
    hold on
    [tx2, x2] = generate(xsent, 50);
    [ty2, y2] = generate(ysent, 50);
    [tz2, z2] = generate(zsent, 50);
    [tth2, th2] = generate(thsent, 50);
    [tgrip2, grip2] = generate(gripsent, 50);
    plot(tx2, x2, '-.')
    plot(ty2, y2, '-.')
    plot(tz2, z2, '-.')
    plot(tth2, th2, '-.')
    plot(tgrip2,grip2, '-.')
    title('Planned Trajectories')
    legend('x', 'y', 'z', 'theta','grip')
    xlabel('Time (s)')
    
    subplot(224)
    axis square
    hold on
    grid on
    view(az,el)
    plot3(x2,y2,z2, ':')
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('Planned Trajectories')
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

function [tx, x] = generate(data, samples)
    % data has 6 cols: a3 a2 a1 a0 ts tf
    % data rows are all the polys sent
    [length, ~] = size(data);
    x = zeros(1, length*samples);
    tx = zeros(1, length*samples);
    for i = 1:length
        a3 = data(i,1);
        a2 = data(i,2);
        a1 = data(i,3);
        a0 = data(i,4);
        ts = data(i,5);
        tf = data(i,6);
        for j = 1:samples
            idx = (i-1)*samples + j;
            t = j/samples*(tf-ts) + ts;
            x(idx) = a3*t.^3 + a2*t.^2 + a1*t + a0;
            tx(idx) = t;
        end
    end
end