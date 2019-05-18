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
serial = selectSerial();

%% main loop
running = 1;
while running
    disp('1 - send trajectory to position from current')
    disp('2 - select a trajectory to send')
    disp('3 - plot all trajectories stored on device')
    disp('4 - send multiple trajectories and run position control')
    disp('5 - run position control on stored trajectories')
    disp('6 - run velocity control on stored trajectories')
    disp('7 - passively read joints and EE position')
    disp('8 - toggle debugging (on by default)')
    disp('9 - plot saved debug data')
    disp('0 - quit')

    user = input('>');

    if user == 1
        disp('Send trajectory to position from current...')
        disp('Not implemented!')
    elseif user == 2
        [xsent, ysent, zsent, thsent, gripsent] = chooseAndSendTrajectory(serial);
    elseif user == 3
        plotStoredTrajectories(serial, xsent, ysent, zsent, thsent, gripsent);
    elseif user == 4
        multipleTrajectories(serial);
    elseif user == 5
        runControl(serial, "PC");
    elseif user == 6
        runControl(serial, "VC");
    elseif user == 7
        readJoints(serial);
    elseif user == 8
        toggleDebugging(serial);
    elseif user == 9
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

