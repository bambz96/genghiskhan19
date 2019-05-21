clear serial % remove any serial that is hanging around and fucking shit up
close all
%% init
xsent = 0;
ysent = 0;
zsent = 0;
thsent = 0;
gripsent = 0;

% position control debugging
global pc_time; pc_time = [];
global pc_xr; pc_xr = [];
global pc_yr; pc_yr = [];
global pc_zr; pc_zr = [];
global pc_xm; pc_xm = [];
global pc_ym; pc_ym = [];
global pc_zm; pc_zm = [];
% velocity control debugging
global vc_time; vc_time = [];
global vc_xr; vc_xr = [];
global vc_yr; vc_yr = [];
global vc_zr; vc_zr = [];
global vc_xm; vc_xm = [];
global vc_ym; vc_ym = [];
global vc_zm; vc_zm = [];
global vc_xe; vc_xe = [];
global vc_ye; vc_ye = [];
global vc_ze; vc_ze = [];
global vc_xdr; vc_xdr = [];
global vc_ydr; vc_ydr = [];
global vc_zdr; vc_zdr = [];

global vc_q1r; vc_q1r = [];
global vc_q2r; vc_q2r = [];
global vc_q3r; vc_q3r = [];
global vc_q1dr; vc_q1dr = [];
global vc_q2dr; vc_q2dr = [];
global vc_q3dr; vc_q3dr = [];
global vc_q1m; vc_q1m = [];
global vc_q2m; vc_q2m = [];
global vc_q3m; vc_q3m = [];
global vc_q1dm; vc_q1dm = [];
global vc_q2dm; vc_q2dm = [];
global vc_q3dm; vc_q3dm = [];
global vc_q1dc; vc_q1dc = [];
global vc_q2dc; vc_q2dc = [];
global vc_q3dc; vc_q3dc = [];

global vc_q1pwm; vc_q1pwm = [];
global vc_q2pwm; vc_q2pwm = [];
global vc_q3pwm; vc_q3pwm = [];
global vc_q1i; vc_q1i = [];
global vc_q2i; vc_q2i = [];
global vc_q3i; vc_q3i = [];

global debugging; debugging = 1; % debugging mode on by default
%% select serial port
disp('Available COM ports:')
ports = seriallist();
options = string(zeros(1, length(ports)));
for i = 1:length(ports)
    options(i) = string(i) +':' + ports(i);
end
disp(join(options))
i = input('Select option:');
serial = serial(ports(i),'BAUD',57600);
%% pause to make sure it's opened
fopen(serial);
pause(1);
disp('Connected to '+ports(i))
%% main loop
running = 1;
while running
    disp('1 - select a velocity signal to use')
    disp('2 - toggle debugging (on by default)')
    disp('3 - plot saved debug data')
    disp('0 - quit')

    user = input('>');

    if user == 1
        selectVelocitySignal(serial);
    elseif user == 2
        toggleDebugging(serial);
    elseif user == 3
        plotDebugData();
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
function selectVelocitySignal(serial)
    disp('Select a velocity signal to use:')
    disp('1 - square')
    disp('2 - square chirp')
    disp('3 - sinusoid')
    select = input('>');
    if select == 1
        cmd = "T1";
    elseif select == 2
        cmd = "T2";
    elseif select == 3
        cmd = "T3";
    end
    fprintf(serial, cmd);
    reply = strtrim(fscanf(serial));
    if ~strcmp(join(string(reply)), cmd)
        disp("Device did not reply correctly, expected 'Tn', got: "+join(string(reply)));
        fclose(serial);
        delete(serial);
        return;
    else
        disp('Device agreed to run velocity signal.')
    end
    
    % reads until receives "DONE"
    readVelocityControlDebugging(serial)
    
    
    
    
end

function readVelocityControlDebugging(serial)
    global vc_time
    global vc_xr vc_yr vc_zr
    global vc_xm vc_ym vc_zm
%     global vc_xe vc_ye vc_ze
%     global vc_xdr vc_ydr vc_zdr
    global vc_q1r vc_q2r vc_q3r
    global vc_q1dr vc_q2dr vc_q3dr
    global vc_q1m vc_q2m vc_q3m
    global vc_q1dm vc_q2dm vc_q3dm
    global vc_q1dc vc_q2dc vc_q3dc
    global vc_q1pwm vc_q2pwm vc_q3pwm
    global vc_q1i vc_q2i vc_q3i
    done = 0;
    while ~done
        data = strtrim(fscanf(serial));
        if ~strcmp(data, "DONE")
            data = sscanf(data, '%f');
            t = data(1)/1000;
            q1m = data(1);
            q2m = data(2);
            q3m = data(3);
            q1dc = data(4);
            q2dc = data(5);
            q3dc = data(6);
            q1dm = data(7);
            q2dm = data(8);
            q3dm = data(9);
%             xr = data(17);
%             yr = data(18);
%             zr = data(19);
%             xm = data(20);
%             ym = data(21);
%             zm = data(22);
%             q1pwm = data(20);
%             q2pwm = data(21);
%             q3pwm = data(22);
%             q1i = data(23);
%             q2i = data(24);
%             q3i = data(25);
            vc_time = [vc_time t];
%             vc_q1r = [vc_q1r q1r];
%             vc_q2r = [vc_q2r q2r];
%             vc_q3r = [vc_q3r q3r];
            vc_q1m = [vc_q1m q1m];
            vc_q2m = [vc_q2m q2m];
            vc_q3m = [vc_q3m q3m];
%             vc_q1dr = [vc_q1dr q1dr];
%             vc_q2dr = [vc_q2dr q2dr];
%             vc_q3dr = [vc_q3dr q3dr];
            vc_q1dc = [vc_q1dc q1dc];
            vc_q2dc = [vc_q2dc q2dc];
            vc_q3dc = [vc_q3dc q3dc];
            vc_q1dm = [vc_q1dm q1dm];
            vc_q2dm = [vc_q2dm q2dm];
            vc_q3dm = [vc_q3dm q3dm];
%             vc_xr = [vc_xr xr];
%             vc_yr = [vc_yr yr];
%             vc_zr = [vc_zr zr];
%             vc_xm = [vc_xm xm];
%             vc_ym = [vc_ym ym];
%             vc_zm = [vc_zm zm];
%             vc_q1pwm = [vc_q1pwm q1pwm];
%             vc_q2pwm = [vc_q2pwm q2pwm];
%             vc_q3pwm = [vc_q3pwm q3pwm];
%             vc_q1i = [vc_q1i q1i];
%             vc_q2i = [vc_q2i q2i];
%             vc_q3i = [vc_q3i q3i];
        else
            return
        end
    end
end


function [xdata, ydata, zdata, thdata, gripdata] = chooseAndSendTrajectory(serial)
    disp('Select a trajectory to send:')
    disp('1 - long test trajectory')
    disp('2 - short test trajectory')
    disp('3 - pick and place 1st block')
    select = input('>');
    if select == 1
        [length, xdata, ydata, zdata, thdata, gripdata] = create_test_trajectory(1);
    elseif select == 2
        [length, xdata, ydata, zdata, thdata, gripdata] = create_test_trajectory(2);
    elseif select == 3
        [length, data] = pickAndPlace();
        xdata = data(:,:,1);
        ydata = data(:,:,2);
        zdata = data(:,:,3);
        thdata = data(:,:,4);
        gripdata = data(:,:,5);
    end
    sendTrajectory(serial, length, xdata, ydata, zdata, thdata, gripdata);
end



function [xdata, ydata, zdata, thdata, gripdata] = sendTrajectory(serial, length, xdata, ydata, zdata, thdata, gripdata)
    %% send command N, indicating about to send N polys
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
    %% send number of polys/row about to be sent
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
    errors = 0;
    disp('Send x')
    errors = errors + sendRow(serial, xdata);
    disp('Send y')
    errors = errors + sendRow(serial, ydata);
    disp('Send z')
    errors = errors + sendRow(serial, zdata);
    disp('Send theta')
    errors = errors + sendRow(serial, thdata);
    disp('Send grip')
    errors = errors + sendRow(serial, gripdata);
    toc
    if errors > 0
        disp('*********************************************************')
        disp('WARNING - SERIAL COMMUNICATION ERRORS')
        disp('Number of coefficient mismatches: '+string(errors))
        disp('*********************************************************')
    end
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

function multipleTrajectories(serial)
    disp('Send multiple trajectories and run position control...')

    disp('Select a motion plan to send:')
    disp('1 - build tower')
    select = input('>');
    if select == 1
        [nchunks, chunks] = createMotionPlan();
    end

    start_time = tic;

    for i = 1:nchunks
        data = chunks(:,:,:,i);
        xdata = data(:,:,1);
        ydata = data(:,:,2);
        zdata = data(:,:,3);
        thdata = data(:,:,4);
        gripdata = data(:,:,5);
        [length,~,~] = size(data);

        sendTrajectory(serial, length, xdata, ydata, zdata, thdata, gripdata);

        disp('Chunk '+string(i)+' of '+string(nchunks)+' sent.')
        success = runControl(serial, "PC", i);
        if ~success
            return
        end
    end

    end_time = toc(start_time);
    disp('Duration of motion plan: '+string(end_time)+'s');
end

function success = runControl(serial, type, chunk_i)
    % type is "PC" or "VC"
    disp('Run control on stored trajectories...')
    fprintf(serial, type);
    received = strtrim(fscanf(serial));
    if strcmp(received, type)
        if strcmp(type, "PC")
            disp('Beginning position control.')
        elseif strcmp(type, "VC")
            disp('Beginning velocity control.')
        end
    else
        disp('Device did not respond correctly: ' + join(string(received)))
    end

    % assume success, set false (0) if
    success = 1;

    global debugging
    if debugging
        % will read debug output until receives "DONE" from device
        if strcmp(type, "PC")
            readPositionControlDebugging(serial)
        elseif strcmp(type, "VC")
            readVelocityControlDebugging(serial)
        end
    else
        % wait for device to confirm position control completed planned trajectories
        while get(serial, 'BytesAvailable') == 0
        end

        received = strtrim(fscanf(serial));
        if strcmp(received, "DONE")
            if nargin == 2 % no chunk_i input
                disp('Path completed.')
            elseif nargin == 3
                disp('Chunk '+string(chunk_i)+' completed.')
            end
        else
            disp('Device did not respond correctly: ' + received)
            disp('Stopping operation.')
        end
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
end

function errors = sendRow(serial, data)
    [length,~] = size(data);
    errors = 0;
    for i = 1:length
        send = join(string(data(i, :)));
        disp("send: " + send);
        fprintf(serial, send);
        reply = strtrim(fscanf(serial));
        disp("recv: " + reply);

        values = sscanf(reply, '%f');
        for j = 1:6
            a = values(j);
            b = data(i,j);
            e = abs(a-b);
            if e > 1e-3
                errors = errors + 1;
                disp('ERROR of '+string(e)+': '+string(a)+' '+string(b))
            end
        end
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

function readPositionControlDebugging(serial)
    global pc_time
    global pc_xr pc_yr pc_zr
    global pc_xm pc_ym pc_zm
    done = 0;
    while ~done
        data = strtrim(fscanf(serial));
        if ~strcmp(data, "DONE")
            data = sscanf(data, '%f');
            t = data(1)/1000;
            xr = data(2);
            yr = data(3);
            zr = data(4);
            xm = data(5);
            ym = data(6);
            zm = data(7);
            pc_time = [pc_time t];
            pc_xr = [pc_xr xr];
            pc_yr = [pc_yr yr];
            pc_zr = [pc_zr zr];
            pc_xm = [pc_xm xm];
            pc_ym = [pc_ym ym];
            pc_zm = [pc_zm zm];
        else
            return
        end
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

function plotDebugData()
    global pc_time
    global pc_xr pc_yr pc_zr
    global pc_xm pc_ym pc_zm
    global vc_time
    global vc_xr vc_yr vc_zr
    global vc_xm vc_ym vc_zm
    global vc_xe vc_ye vc_ze
    global vc_xdr vc_ydr vc_zdr
    global vc_q1r vc_q2r vc_q3r
    global vc_q1dr vc_q2dr vc_q3dr
    global vc_q1m vc_q2m vc_q3m
    global vc_q1dm vc_q2dm vc_q3dm
    global vc_q1dc vc_q2dc vc_q3dc
    global vc_q1pwm vc_q2pwm vc_q3pwm
    global vc_q1i vc_q2i vc_q3i

    % default line colours
    c1 = [0, 0.4470, 0.7410]; % blue
    c2 = [0.8500, 0.3250, 0.0980]; % orange
    c3 = [0.9290, 0.6940, 0.1250]; % yellow
    c4 = [0.4940, 0.1840, 0.5560]; % purple
    c5 = [0.4660, 0.6740, 0.1880]; % green
    c6 = [0.6350, 0.0780, 0.1840]; % red

    if ~isempty(pc_time)
        % position control plots
        pc_f = 1/mean(diff(pc_time));

        figure
        subplot(121)
        hold on
        plot(pc_time, pc_xr)
        plot(pc_time, pc_yr)
        plot(pc_time, pc_zr)
        title('Position Control, f='+string(pc_f)+'Hz')
        legend('x', 'y', 'z')
        xlabel('Time (s)')

        subplot(122)
        axis square
        hold on
        grid on
        view(15,15)
        plot3(pc_xr, pc_yr, pc_zr, ':')
        plot3(pc_xm, pc_ym, pc_zm)
        title('Trajectory')
        legend('Reference', 'Measured')
        xlabel('x')
        ylabel('y')
        zlabel('z')
    end
    if ~isempty(vc_time)
        % velocity control plots
        vc_f = 1/mean(diff(vc_time));

        figure
        subplot(311)
        hold on
        plot(vc_time, vc_q1m)
        plot(vc_time, vc_q1dc, ':')
        plot(vc_time, vc_q1dm)
        title('Q1 Velocities, f='+string(vc_f)+'Hz')
        legend('qm', 'qdc', 'qdm')
        
        subplot(312)
        hold on
        plot(vc_time, vc_q2m) 
        plot(vc_time, vc_q2dc, ':')
        plot(vc_time, vc_q2dm)
        title('Q2 Velocities')
        legend('qm', 'qdc', 'qdm')
        
        subplot(313)
        hold on
        plot(vc_time, vc_q3m)
        plot(vc_time, vc_q3dc, ':')
        plot(vc_time, vc_q3dm)
        title('Q3 Velocities')
        legend('qm', 'qdc', 'qdm')
    end
end

function toggleDebugging(serial)
    global debugging
    fprintf(serial, "D");
    received = strtrim(fscanf(serial));
    if strcmp(received, "D")
        debugging = ~debugging;
        disp('Toggled debugging, current state: '+debugging)
    else
        disp('Device did not respond correctly: '+received)
    end
end
