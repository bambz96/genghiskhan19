close all
%% robot dimensions
dim.h = 0.15;
dim.l1 = 0.4;
dim.l2 = 0.25;
dim.l3 = 0.05;
dim.l4 = 0.05;
%% poses
x0 = 0.3; % tower position (corner)
y0 = 0.3; % tower position (corner)
ph = [0.5 0 0 0]; % block pickup position
p0 = ph;
%% loop for each pose
state = 1; % block number currently delivering
has_block = 1; % boolean
while state < 3*5 % build 5 layers
    %% time
    ti = 0;
    tvia = 0.15;
    tf = 0.3;
    dt = 0.01;
    t = ti:dt:tf;
    %% get current and next pose
    k = 1;
    valid = 0;
    while ~valid
        if has_block
            [x, y, z, theta] = block_pos(state, x0, y0);
            has_block = 0;
            state = state + 1;
        else
            x = ph(1); y = ph(2); z = ph(3); theta = ph(4);
            has_block = 1;
        end
        pf = [x, y, z, theta];
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
        end
    end
    %% animate robot
    hold on
    grid on
%     figure(gcf) % bring figure window to front
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
    end
    p0 = pf;
end