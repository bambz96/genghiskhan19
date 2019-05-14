function [t1, t2, t3, t4, t5, valid] = inverse_kin(x, y, z, theta, dim)
    % find absolute joint angles to target position (x,y,z)
    % set end effector have rotation about z0 by theta

    h = dim.h; % base height
    l1 = dim.l1; % 1st arm length
    l2 = dim.l2; % 2nd arm length
    l3 = dim.l3; % 3rd arm length (end effector)
    l4 = dim.l4; % 4th arm length (end effector)

    % working variables
    zp = z + l3 + l4 - h;
    r = sqrt(x^2 + y^2);
    c = sqrt(r^2 + zp^2);

    % thetas (absolute joint angles)
    var t1; % base rotation
    var t2; % first arm rotation
    var t3; % second arm rotation
    var t4; % effector rotation
    var t5; % effector vertical rotation

    % base rotation
    t1 = atand(y/x);

    % second arm angle
    num = - r^2 - zp^2 + l1^2 + l2^2;
    den = 2*l1*l2;
    t3 = acosd(num/den) - 180;

    % first arm angle
    a = atand(zp/r);
    b = acosd( (l1^2 + c^2 - l2^2) / (2*l1*c) );
    t2 = a + b;

    % end effector angle (always vertical constraint)
    t4 = - t2 - t3;

    % end effector rotation around vertical
    t5 = theta - t1;
    
    % if all angles aren't real, use arbitrary real angles and set valid=0
    if isreal([t1 t2 t3 t4 t5]) == 0
        t1 = 0; t2 = 0; t3 = 0; t4 = 0; t5 = 0;
        valid = 0;
    else
        % else assume valid, doesn't consider joint limits
        valid = 1;
    end
end