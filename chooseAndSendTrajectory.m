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