function readPositionControlDebugging(serial)
    global pc_time
    global pc_xr pc_yr pc_zr
    global pc_xm pc_ym pc_zm
    done = 0;
    while ~done
        data = strtrim(fscanf(serial));
        if ~strcmp(data, "DONE")
            data = sscanf(data, '%f');
            t = data(1)/1000;
            xr = data(2);
            yr = data(3);
            zr = data(4);
            xm = data(5);
            ym = data(6);
            zm = data(7);
            pc_time = [pc_time t];
            pc_xr = [pc_xr xr];
            pc_yr = [pc_yr yr];
            pc_zr = [pc_zr zr];
            pc_xm = [pc_xm xm];
            pc_ym = [pc_ym ym];
            pc_zm = [pc_zm zm];
        else
            return
        end
    end
end