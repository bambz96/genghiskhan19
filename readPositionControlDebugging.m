function readPositionControlDebugging(serial)
    global pc_time
    global pc_xr pc_yr pc_zr
    global pc_xm pc_ym pc_zm
    global vc_q1r vc_q2r vc_q3r
    global vc_q1m vc_q2m vc_q3m
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
            q1r = data(8);
            q2r = data(9);
            q3r = data(10);
            q1m = data(11);
            q2m = data(12);
            q3m = data(13);
            pc_time = [pc_time t];
            pc_xr = [pc_xr xr];
            pc_yr = [pc_yr yr];
            pc_zr = [pc_zr zr];
            pc_xm = [pc_xm xm];
            pc_ym = [pc_ym ym];
            pc_zm = [pc_zm zm];
            vc_q1r = [vc_q1r q1r];
            vc_q2r = [vc_q2r q2r];
            vc_q3r = [vc_q3r q3r];
            vc_q1m = [vc_q1m q1m];
            vc_q2m = [vc_q2m q2m];
            vc_q3m = [vc_q3m q3m];
        else
            return
        end
    end
end