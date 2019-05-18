function errors = sendRow(serial, data)
    [length,~] = size(data);
    errors = 0;
    for i = 1:length
        send = join(string(data(i, :)));
        disp("send: " + send);
        fprintf(serial, send);
        reply = strtrim(fscanf(serial));
        disp("recv: " + reply);

        values = sscanf(reply, '%f');
        for j = 1:6
            a = values(j);
            b = data(i,j);
            e = abs(a-b);
            if e > 1e-3
                errors = errors + 1;
                disp('ERROR of '+string(e)+': '+string(a)+' '+string(b))
            end
        end
    end
end