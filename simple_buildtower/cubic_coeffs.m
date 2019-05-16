function coeffs = cubic_coeffs(x0, x0d, xf, xfd, tf)
    % Find the coefficients for a cubic polynomial that fit between the
    % initial and final positions and velocities. Duration tf.
    % x = a3*t.^3 + a2*t.^2 + a1*t + a0;
    % xd = 3*a3*t.^2 + 2*a2*t + a1;
    A = [1 0 0 0; 0 1 0 0; 1 tf tf^2 tf^3; 0 1 2*tf 3*tf^2];

    b = inv(A)*[x0; x0d; xf; xfd];

    a3 = myShrink(b(4));
    a2 = myShrink(b(3));
    a1 = myShrink(b(2));
    a0 = myShrink(b(1));
    
    coeffs = [a3 a2 a1 a0 0 tf];

end

function val = myShrink(x)
    if abs(x) < 1e-4
        val = 0;
    else
        val = x;
    end
end