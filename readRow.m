function [t, d] = readRow(serial, length, path_res)
    disp('Waiting for data...')
    i = 1;
    t = zeros(1, path_res*length);
    d = zeros(1, path_res*length);
    while i <= path_res*length
        data = strtrim(fscanf(serial));
        res = regexp(data, '[+-]?\d+\.?\d*','match');
        t(i) = str2double(res{1});
        d(i) = str2double(res{2});
        i = i + 1;
    end
end