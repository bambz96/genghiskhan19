function readVelocityControlDebugging(serial)
    global vc_time
    global vc_xr vc_yr vc_zr
    global vc_xm vc_ym vc_zm
%     global vc_xe vc_ye vc_ze
%     global vc_xdr vc_ydr vc_zdr
    global vc_q1r vc_q2r vc_q3r
    global vc_q1dr vc_q2dr vc_q3dr
    global vc_q1m vc_q2m vc_q3m
    global vc_q1dm vc_q2dm vc_q3dm
    global vc_q1dc vc_q2dc vc_q3dc
    global vc_q1pwm vc_q2pwm vc_q3pwm
    global vc_q1i vc_q2i vc_q3i
    done = 0;
    while ~done
        data = strtrim(fscanf(serial));
        if ~strcmp(data, "DONE")
            data = sscanf(data, '%f');
            t = data(1)/1000;
            q1r = data(2);
            q2r = data(3);
            q3r = data(4);
            q1m = data(5);
            q2m = data(6);
            q3m = data(7);
            q1dr = data(8);
            q2dr = data(9);
            q3dr = data(10);
            q1dc = data(11);
            q2dc = data(12);
            q3dc = data(13);
            q1dm = data(14);
            q2dm = data(15);
            q3dm = data(16);
            xr = data(17);
            yr = data(18);
            zr = data(19);
            xm = data(20);
            ym = data(21);
            zm = data(22);
%             q1pwm = data(20);
%             q2pwm = data(21);
%             q3pwm = data(22);
%             q1i = data(23);
%             q2i = data(24);
%             q3i = data(25);
            vc_time = [vc_time t];
            vc_q1r = [vc_q1r q1r];
            vc_q2r = [vc_q2r q2r];
            vc_q3r = [vc_q3r q3r];
            vc_q1m = [vc_q1m q1m];
            vc_q2m = [vc_q2m q2m];
            vc_q3m = [vc_q3m q3m];
            vc_q1dr = [vc_q1dr q1dr];
            vc_q2dr = [vc_q2dr q2dr];
            vc_q3dr = [vc_q3dr q3dr];
            vc_q1dc = [vc_q1dc q1dc];
            vc_q2dc = [vc_q2dc q2dc];
            vc_q3dc = [vc_q3dc q3dc];
            vc_q1dm = [vc_q1dm q1dm];
            vc_q2dm = [vc_q2dm q2dm];
            vc_q3dm = [vc_q3dm q3dm];
            vc_xr = [vc_xr xr];
            vc_yr = [vc_yr yr];
            vc_zr = [vc_zr zr];
            vc_xm = [vc_xm xm];
            vc_ym = [vc_ym ym];
            vc_zm = [vc_zm zm];
%             vc_q1pwm = [vc_q1pwm q1pwm];
%             vc_q2pwm = [vc_q2pwm q2pwm];
%             vc_q3pwm = [vc_q3pwm q3pwm];
%             vc_q1i = [vc_q1i q1i];
%             vc_q2i = [vc_q2i q2i];
%             vc_q3i = [vc_q3i q3i];
        else
            return
        end
    end
end