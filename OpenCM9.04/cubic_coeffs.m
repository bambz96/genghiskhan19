function [a3, a2, a1, a0] = cubic_coeffs(x0, x0d, xf, xfd, tf)
    % Find the coefficients for a cubic polynomial that fit between the
    % initial and final positions and velocities. Duration tf.
    % x = a3*t.^3 + a2*t.^2 + a1*t + a0;
    % xd = 3*a3*t.^2 + 2*a2*t + a1;
    A = [1 0 0 0; 0 1 0 0; 1 tf tf^2 tf^3; 0 1 2*tf 3*tf^2];

    b = inv(A)*[x0; x0d; xf; xfd];

    a3 = b(4);
    a2 = b(3);
    a1 = b(2);
    a0 = b(1);
end