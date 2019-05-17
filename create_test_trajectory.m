function [length, xdata, ydata, zdata, thdata, gripdata] = create_test_trajectory()

% grip and ungrip motor angles in radians
grip = 1.015976119;
ungrip = 0.422257077;

% metres, degrees, degrees, ?, seconds
% x y z theta grip ts
s = 0.1; % speed up
p = [
    0.2 0 0.3 0 ungrip 0*s;
    0.15 -0.2 0.05 0 ungrip 2*s;
    0.15 -0.2 0.05 0 ungrip 7*s;
    0.2 0 0.05 0 ungrip 8*s;
    0.2 0 0.05 0 ungrip 13*s;
    0.125 0.25 0.05 0 ungrip 14*s;
    0.125 0.25 0.05 0 ungrip 19*s;
    0.2 0 0.05 0 ungrip 20*s;
    0.2 0 0.05 0 ungrip 25*s;
    0.15 -0.2 0.05 0 ungrip 30*s;
    0.15 -0.2 0.05 0 ungrip 35*s;
    0.2 0 0.05 0 ungrip 36*s;
    0.2 0 0.05 0 ungrip 41*s;
    0.2 0 0.3 0 ungrip 43*s
];

disp(p)

[length, ~] = size(p);
length = length - 1;

xdata = zeros(length, 6);
ydata = zeros(length, 6);
zdata = zeros(length, 6);
thdata = zeros(length, 6);
gripdata = zeros(length, 6);

%  current to p(1), the first pose, in 3 seconds

for i = 1:length
    x0 = p(i,1); xf = p(i+1,1);
    y0 = p(i,2); yf = p(i+1,2);
    z0 = p(i,3); zf = p(i+1,3);
    th0 = p(i,4)*pi/180; thf = p(i+1,4)*pi/180;
    gr0 = p(i,5); grf = p(i+1,5);
    ts = p(i,6); tf = p(i+1,6);
    xdata(i,:) = cubic_coeffs(x0, 0, xf, 0, ts, tf);
    ydata(i,:) = cubic_coeffs(y0, 0, yf, 0, ts, tf);
    zdata(i,:) = cubic_coeffs(z0, 0, zf, 0, ts, tf);
    thdata(i,:) = cubic_coeffs(th0, 0, thf, 0, ts, tf);
    gripdata(i,:) = cubic_coeffs(gr0, 0, grf, 0, ts, tf);
end

end

function coeffs = cubic_coeffs(x0, x0d, xf, xfd, ts, tf)
    % Find the coefficients for a cubic polynomial that fit between the
    % initial and final positions and velocities. Duration tf.
    % x = a3*t.^3 + a2*t.^2 + a1*t + a0;
    % xd = 3*a3*t.^2 + 2*a2*t + a1;
    A = [1 ts ts^2 ts^3; 0 1 2*ts 3*ts^2; 1 tf tf^2 tf^3; 0 1 2*tf 3*tf^2];

    b = inv(A)*[x0; x0d; xf; xfd];

    a3 = myShrink(b(4));
    a2 = myShrink(b(3));
    a1 = myShrink(b(2));
    a0 = myShrink(b(1));
    
    coeffs = [a3 a2 a1 a0 ts tf];

end

function val = myShrink(x)
    if abs(x) < 1e-4
        val = 0;
    else
        val = x;
    end
end