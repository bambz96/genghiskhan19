function [x, xd] = cubic(t, x0, x0d, xf, xfd)
    [a3, a2, a1, a0] = cubic_coeffs(x0,x0d,xf,xfd,t(length(t)));
    x = a3*t.^3 + a2*t.^2 + a1*t + a0;
    xd = 3*a3*t.^2 + 2*a2*t + a1;
end