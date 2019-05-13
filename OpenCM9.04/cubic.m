function [x, xd] = cubic(t, x0, x0d, xf, xfd)
    c = cubic_coeffs(x0,x0d,xf,xfd,t(length(t)));
    a3 = c(1);
    a2 = c(2);
    a1 = c(3);
    a0 = c(4);
    x = a3*t.^3 + a2*t.^2 + a1*t + a0;
    xd = 3*a3*t.^2 + 2*a2*t + a1;
end