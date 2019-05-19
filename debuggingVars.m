function debuggingVars()
% position control debugging
global pc_time; pc_time = [];
global pc_xr; pc_xr = [];
global pc_yr; pc_yr = [];
global pc_zr; pc_zr = [];
global pc_xm; pc_xm = [];
global pc_ym; pc_ym = [];
global pc_zm; pc_zm = [];
% velocity control debugging
global vc_time; vc_time = [];
global vc_xr; vc_xr = [];
global vc_yr; vc_yr = [];
global vc_zr; vc_zr = [];
global vc_xm; vc_xm = [];
global vc_ym; vc_ym = [];
global vc_zm; vc_zm = [];
global vc_xe; vc_xe = [];
global vc_ye; vc_ye = [];
global vc_ze; vc_ze = [];
global vc_xdr; vc_xdr = [];
global vc_ydr; vc_ydr = [];
global vc_zdr; vc_zdr = [];

global vc_q1r; vc_q1r = [];
global vc_q2r; vc_q2r = [];
global vc_q3r; vc_q3r = [];
global vc_q1dr; vc_q1dr = [];
global vc_q2dr; vc_q2dr = [];
global vc_q3dr; vc_q3dr = [];
global vc_q1m; vc_q1m = [];
global vc_q2m; vc_q2m = [];
global vc_q3m; vc_q3m = [];
global vc_q1dm; vc_q1dm = [];
global vc_q2dm; vc_q2dm = [];
global vc_q3dm; vc_q3dm = [];
global vc_q1dc; vc_q1dc = [];
global vc_q2dc; vc_q2dc = [];
global vc_q3dc; vc_q3dc = [];

global vc_q1pwm; vc_q1pwm = [];
global vc_q2pwm; vc_q2pwm = [];
global vc_q3pwm; vc_q3pwm = [];
global vc_q1i; vc_q1i = [];
global vc_q2i; vc_q2i = [];
global vc_q3i; vc_q3i = [];

global debugging; debugging = 1; % debugging mode on by default
end