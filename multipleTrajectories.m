function multipleTrajectories(serial,x,y,theta,loadSide,speed)
    disp('Send multiple trajectories and run position control...')

%     disp('Select a motion plan to send:')
%     disp('1 - build tower')
%     select = input('>');
%     if select == 1
        [nchunks, chunks] = createMotionPlan(x,y,theta,loadSide,speed, 54);
%     endr

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