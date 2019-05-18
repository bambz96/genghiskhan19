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