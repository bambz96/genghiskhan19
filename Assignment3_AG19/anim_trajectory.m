close all
%% save plots of key poses
save = 0;
%% robot dimensions
dim.h = 0.15;
dim.l1 = 0.4;
dim.l2 = 0.25;
dim.l3 = 0.05;
dim.l4 = 0.05;
%% poses
p = [
    0.5 0 0.05 0;
    0.4 0.4 0.1 30;
    0.2 0.1 0.3 -45;
    0.4 0.4 0.6 0;
    0.4 0.4 0.2 0;
    0.3 0.3 0 0;
    ].';
%% loop for each pose
j = 1;
while j < length(p)
    %% time
    ti = 0;
    tvia = 0.6;
    tf = 1.2;
    dt = 0.01;
    t = ti:dt:tf;
    %% get current and next pose
    p0 = p(:,j);
    k = 1;
    valid = 0;
    while ~valid
        pf = p(:,j+k);
        poses = zeros(5, length(t)); % store joint angles for animation
        %% calculate via
        vx = (pf(1) - p0(1))*0.5 + p0(1);
        vy = (pf(2) - p0(2))*0.5 + p0(2);
        % only approach from above when end point is above start point
        % otherwise, remain horizontal
        if (pf(3) - p0(3)) > 0
            vz = pf(3)*1.15;
        else
            vz = p0(3);
        end
        via = [vx vy vz];
        %% cubic interpolation
        [x, xd] = cubic_via(p0(1), 0, pf(1), 0, via(1), tvia, tf);
        [y, yd] = cubic_via(p0(2), 0, pf(2), 0, via(2), tvia, tf);
        [z, zd] = cubic_via(p0(3), 0, pf(3), 0, via(3), tvia, tf);
        [a, ad] = cubic(t, p0(4), 0, pf(4), 0);
        for i = 1:length(t)
            [t1, t2, t3, t4, t5, valid] = inverse_kin(x(i), y(i), z(i), a(i), dim);
            if valid
                poses(:, i) = [t1 t2 t3 t4 t5].';
            else
                % end validation loop
                fprintf('Not a valid pose at time %.2fs.\n', t(i));
                % fprintf('[%.2f %.2f %.2f %.2f] is not a valid pose \n', t(i));
                break
            end
        end
        if ~valid
            disp('Moving to next pose.')
            plot3(x, y, z, 'lineStyle', ':', 'color', 'r')
            scatter3([p0(1) pf(1) via(1)], [p0(2) pf(2) via(2)], [p0(3) pf(3) via(3)])
            k = k+1;
            if save
                gcf.PaperPositionMode = 'auto';
                print(sprintf('%d_error', j),'-dpng') % '-r0' makes it same pixels as on screen
            end
            pause(2.5);
        end
    end
    % if pose(s) are skipped then k>1, need to skip starting pose j as well
    j = j + k;
    %% animate robot
    hold on
    grid on
    %figure(gcf) % bring figure window to front
    for i = 1:length(x)
        cla
        plot3(x, y, z)
        scatter3([p0(1) pf(1) via(1)], [p0(2) pf(2) via(2)], [p0(3) pf(3) via(3)])
        % absolute joint angles for drawing
        t1 = poses(1, i);
        t2 = poses(2, i);
        t3 = poses(3, i);
        t4 = poses(4, i);
        t5 = poses(5, i);
        draw_robot(t1, t2, t3, t4, t5, dim, 1, 30, 30, 0.75);
        pause(dt/10);
        if mod(i, 50) == 0 && save
            gcf.PaperPositionMode = 'auto';
            print(sprintf('%d_%d', j, i),'-dpng') % '-r0' makes it same pixels as on screen
        end
    end
end