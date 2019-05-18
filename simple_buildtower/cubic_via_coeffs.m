function [a3, a2, a1, a0, b3, b2, b1, b0] = ...
    cubic_via_coeffs(x0, x0d, xf, xfd, via, tv, tf)
% Given start/end positions and velocities, a via point, and the time to
% reach the via point and end point, find a path made of two cubics.
% The cubics are continuous at the point where the segments meet.
A = [0 0 0 1 0 0;
    0 0 1 0 0 0;
    tv^3 tv^2 tv 1 0 0;
    3*tv^2 2*tv 1 0 -1 0;
    6*tv 2 0 0 0 -1];
B = [0 0 tv^3 tv^2 tv 1;
    -1 0 3*tv^2 2*tv 1 0;
    0 -1 6*tv 2 0 0;
    0 0 tf^3 tf^2 tf 1;
    0 0 3*tf^2 2*tf 1 0];
    
M = [A zeros(5,4); zeros(5,4) B];
X = [x0 x0d via 0 0 via 0 0 xf xfd].';

b = inv(M)*X;

% polynomial coefficients
a3 = b(1);
a2 = b(2);
a1 = b(3);
a0 = b(4);
b3 = b(7);
b2 = b(8);
b1 = b(9);
b0 = b(10);
end