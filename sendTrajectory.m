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