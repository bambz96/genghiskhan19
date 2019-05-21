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
        plot(pc_time, pc_xr, ':')
        plot(pc_time, pc_yr, ':')
        plot(pc_time, pc_zr, ':')
        plot(pc_time, pc_xm)
        plot(pc_time, pc_ym)
        plot(pc_time, pc_zm)
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
        
        figure;
        hold on
        plot(pc_time, vc_q1r, '--')
        plot(pc_time, vc_q2r, '--')
        plot(pc_time, vc_q3r, '--')
        plot(pc_time, vc_q1m)
        plot(pc_time, vc_q2m)
        plot(pc_time, vc_q3m)
        title('Joint Angles, f='+string(pc_f)+'Hz')
        legend('q1r', 'q2r', 'q3r', 'q1m', 'q2m', 'q3m')
    end
    if ~isempty(vc_time)
        % velocity control plots
        vc_f = 1/mean(diff(vc_time));

        figure
        subplot(211)
        hold on
        plot(vc_time, vc_q1r, '--')
        plot(vc_time, vc_q2r, '--')
        plot(vc_time, vc_q3r, '--')
        plot(vc_time, vc_q1m)
        plot(vc_time, vc_q2m)
        plot(vc_time, vc_q3m)
        title('Joint Angles, f='+string(vc_f)+'Hz')
        legend('q1r', 'q2r', 'q3r', 'q1m', 'q2m', 'q3m')
        
        subplot(212)
        hold on
        plot(vc_time, vc_xr, '--')
        plot(vc_time, vc_yr, '--')
        plot(vc_time, vc_zr, '--')
        plot(vc_time, vc_xm)
        plot(vc_time, vc_ym)
        plot(vc_time, vc_zm)
        title('Task Space Position')
        legend('xr', 'yr', 'zr', 'xm', 'ym', 'zm')

%         subplot(224)
%         hold on
%         plot(vc_time, vc_q1dc)
%         plot(vc_time, vc_q2dc)
%         plot(vc_time, vc_q3dc)
%         plot(vc_time, vc_q1pwm, ':')
%         plot(vc_time, vc_q2pwm, ':')
%         plot(vc_time, vc_q3pwm, ':')
%         plot(vc_time, vc_q1i, '--')
%         plot(vc_time, vc_q2i, '--')
%         plot(vc_time, vc_q3i, '--')
%         title('Control and PWM')
%         legend('q1dc', 'q2dc', 'q3dc', 'pwm1', 'pwm2', 'pwm3', 'i1', 'i2', 'i3')

        figure
        subplot(311)
        hold on
        plot(vc_time, vc_q1dr, '--')
        plot(vc_time, vc_q1dc, ':')
        plot(vc_time, vc_q1dm)
        title('Q1 Velocities')
        legend('r', 'c', 'm')
        
        subplot(312)
        hold on
        plot(vc_time, vc_q2dr, '--')
        plot(vc_time, vc_q2dc, ':')
        plot(vc_time, vc_q2dm)
        title('Q2 Velocities')
        legend('r', 'c', 'm')
        
        subplot(313)
        hold on
        plot(vc_time, vc_q3dr, '--')
        plot(vc_time, vc_q3dc, ':')
        plot(vc_time, vc_q3dm)
        title('Q3 Velocities')
        legend('r', 'c', 'm')

        
    end
end