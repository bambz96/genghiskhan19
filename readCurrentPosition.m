function [x,y,z,thx,thy,thz,grip] = readCurrentPosition(serial)
    disp('Get current EE position...')
    fprintf(serial, 'REE');
    reply = strtrim(fgetl(serial));
    if strcmp(reply, 'REE')
        disp('Device is sending current EE position.')
    else
        disp('Device did not respond correctly: '+string(reply))
        [x, y, z, thx, thy, thz] = zeros(1,7);
        return
    end
    data = fscanf(serial, '%f');
    disp(data);
    x = data(1);
    y = data(2);
    z = data(3);
    thx = data(4);
    thy = data(5);
    thz = data(6);
    grip = data(7);
end

