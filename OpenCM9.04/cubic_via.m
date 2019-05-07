function [x, xd, xdd] = cubic_via(x0, x0d, xf, xfd, via, tv, tf)
% Return the position, velocity and acceleration for the cubic polynomial
% paths joining x0 and xf through the via point.
[a3, a2, a1, a0, b3, b2, b1, b0] = cubic_via_coeffs(x0, x0d, xf, xfd, via, tv, tf);

dt = 0.01;
t = 0:dt:tf;

% find the index of the time closest to tv
i = find(abs(t-tv) < dt);

% first segment
x1 = a3*t.^3 + a2*t.^2 + a1*t + a0;
x1d = 3*a3*t.^2 + 2*a2*t + a1;
x1dd = 6*a3*t + 2*a2;
% second segment
x2 = b3*t.^3 + b2*t.^2 + b1*t + b0;
x2d = 3*b3*t.^2 + 2*b2*t + b1;
x2dd = 6*b3*t + 2*b2;

% join the segments
x = [x1(1:i) x2(i+1:length(t))];
xd = [x1d(1:i) x2d(i+1:length(t))];
xdd = [x1dd(1:i) x2dd(i+1:length(t))];
end