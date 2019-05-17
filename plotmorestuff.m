subplot(311)
hold on
plot(vc_time, vc_xdr)
plot(vc_time, vc_xm)
plot(vc_time, vc_xe)
plot(vc_time, vc_xr)
legend('xdc', 'xm', 'xe')

subplot(312)
hold on
plot(vc_time, vc_ydr)
plot(vc_time, vc_ym)
plot(vc_time, vc_ye)
plot(vc_time, vc_yr)
legend('xdc', 'xm', 'xe')

subplot(313)
hold on
plot(vc_time, vc_zdr)
plot(vc_time, vc_zm)
plot(vc_time, vc_ze)
plot(vc_time, vc_zr)
legend('xdc', 'xm', 'xe')