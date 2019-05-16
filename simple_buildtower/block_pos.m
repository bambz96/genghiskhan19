function [x, y, z, theta] = block_pos(n, x0, y0)

    w = 0.025;
    l = 0.075;
    h = 0.015;

    lvl = ceil(n/3);
    idx = mod(n, 3);
    rot = mod(lvl,2);
    
    if idx == 0
        idx = 3;
    end
    idx = idx - 1;
    disp(idx)
    
    z = (lvl-1) * h + h/2;
    if rot == 0
        x = idx * w + x0 + w/2;
        y = y0 + l/2;
        theta = 90;
    else
        x = x0 + l/2;
        y = idx * w + y0 + w/2;
        theta = 0;
    end
end