clear serial % remove any serial that is hanging around and fucking shit up
close all
%% init
% xsent = 0;
% ysent = 0;
% zsent = 0;
% thsent = 0;
% gripsent = 0;

debuggingVars();
serial = selectSerial();

%% main loop
running = 1;
while running
    disp('1 - read current EE position')
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
        disp(readCurrentPosition(serial));
    elseif user == 2
        [xsent, ysent, zsent, thsent, gripsent] = chooseAndSendTrajectory(serial);
    elseif user == 3
        plotStoredTrajectories(serial, xsent, ysent, zsent, thsent, gripsent);
    elseif user == 4
        multipleTrajectories(serial,200,-100,0,'A','Full Jenghis');
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

