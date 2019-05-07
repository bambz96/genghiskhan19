function vE = draw_robot(t1, t2, t3, t4, t5, dim, doDraw, az, el, zoom)
    % draws robot using forward kinematics
    % with given absolute joint angles ti (degrees)
    % and dimensions h (base height) and li (arm lengths)
    
    h = dim.h; % base height
    l1 = dim.l1; % 1st arm length
    l2 = dim.l2; % 2nd arm length
    l3 = dim.l3; % 3rd arm length
    l4 = dim.l4; % 4th arm length
    
    % limit absolute joint angles (if limited, link colour will change to red)
    [t1, c1] = limit(t1, -90, 90);
    [t2, c2] = limit(t2, 20, 60);
    [t3, c3] = limit(t3, -130, -20);
    [t4, c4] = limit(t4, -60, 80);
    [t5, c5] = limit(t5, -90, 90);
    
    % T from to
    T01 = DH(0, 0, h, t1);
    T12 = DH(0, 90, 0, t2);
    T23 = DH(l1, 0, 0, t3);
    T34 = DH(l2, 0, 0, t4);
    T45 = DH(0, 90, l3, t5);
    T5E = DH(0, 0, l4, 0);

    T02 = T01*T12;
    T03 = T01*T12*T23;
    T04 = T01*T12*T23*T34;
    T05 = T01*T12*T23*T34*T45;
    T0E = T01*T12*T23*T34*T45*T5E;

    % transforming origin to find each link end point
    v = [0 0 0 1]';
    v1 = T01*v;
    v2 = T02*v; % same as v1 (due to pure joint rotation)
    v3 = T03*v;
    v4 = T04*v;
    v5 = T05*v;
    vE = T0E*v;
    vE = vE(1:3);
    
    if doDraw
        hold on
        % draw each link as a line
        w = 1;
        line([0 v1(1)], [0 v1(2)], [0 v1(3)], 'color', c1, 'LineWidth', w);
        line([v2(1) v3(1)], [v2(2) v3(2)], [v2(3) v3(3)], 'color', c2, 'LineWidth', w);
        line([v3(1) v4(1)], [v3(2) v4(2)], [v3(3) v4(3)], 'color', c3, 'LineWidth', w);
        line([v4(1) v5(1)], [v4(2) v5(2)], [v4(3) v5(3)], 'color', c4, 'LineWidth', w);
        line([v5(1) vE(1)], [v5(2) vE(2)], [v5(3) vE(3)], 'color', c5, 'LineWidth', w);
        % draw line showing end effector orientation
        L = 0.1; % line length
        % t5 + t1 is angle about z0
        x1 = vE(1) - L/2 * cosd(t5 + t1);
        x2 = vE(1) + L/2 * cosd(t5 + t1);
        y1 = vE(2) - L/2 * sind(t5 + t1);
        y2 = vE(2) + L/2 * sind(t5 + t1);
        z1 = vE(3); z2 = z1;
        line([x1 x2], [y1 y2], [z1 z2], 'color', c5, 'LineWidth', w);
        % draw each joint
        w = 3;
        plot3(v1(1), v1(2), v1(3), 'o', 'Color','k', 'MarkerSize',w,'MarkerFaceColor','k')
        plot3(v3(1), v3(2), v3(3), 'o', 'Color','k', 'MarkerSize',w,'MarkerFaceColor','k')
        plot3(v4(1), v4(2), v4(3), 'o', 'Color','k', 'MarkerSize',w,'MarkerFaceColor','k')
        plot3(v5(1), v5(2), v5(3), 'o', 'Color','k', 'MarkerSize',w-1,'MarkerFaceColor','k')
        plot3(vE(1), vE(2), vE(3), 'o', 'Color','k', 'MarkerSize',w,'MarkerFaceColor','k')

        axis square
        view([az el])
        axis(zoom*[0 1 0 1 0 1])
        xlabel('X (m)')
        ylabel('Y (m)')
        zlabel('Z (m)')
    end
end

function [x, c] = limit(val, low, high)
    % constrain value to [low, high]
    % also return colour for link (red if limited, black if unaffected)
    x = val;
    xtest = min(max(val, low), high);
    if xtest ~= val
        c = 'r';
    else
        c = 'k';
    end
end