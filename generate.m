function [tx, x] = generate(data, samples)
    % data has 6 cols: a3 a2 a1 a0 ts tf
    % data rows are all the polys sent
    [length, ~] = size(data);
    x = zeros(1, length*samples);
    tx = zeros(1, length*samples);
    for i = 1:length
        a3 = data(i,1);
        a2 = data(i,2);
        a1 = data(i,3);
        a0 = data(i,4);
        ts = data(i,5);
        tf = data(i,6);
        for j = 1:samples
            idx = (i-1)*samples + j;
            t = j/samples*(tf-ts) + ts;
            x(idx) = a3*t.^3 + a2*t.^2 + a1*t + a0;
            tx(idx) = t;
        end
    end
end