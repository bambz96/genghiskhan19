close all
%% define constraints
p0 = [0.4 0 0.05];
pf = [0.2 0.3 0.3];
v0 = [0 0 0];
vf = [0 0 0];
via = [0.2 0.2 0.1];
%% time
tf = 2; % time to reach end
t = 0:0.01:tf;
%% find paths
[x, xd] = cubic(t, p0(1), v0(1), pf(1), vf(1));
[y, yd] = cubic(t, p0(2), v0(2), pf(2), vf(2));
[z, zd] = cubic(t, p0(3), v0(3), pf(3), vf(3));
%% plot
subplot(211)
hold on
title('Position')
ylabel('Position (m)')
xlabel('Time (s)')
plot(t, x)
plot(t, y)
plot(t, z)
legend('x', 'y', 'z')

subplot(212)
hold on
title('Velocity')
ylabel('Velocity (m/s)')
xlabel('Time (s)')
plot(t, xd)
plot(t, yd)
plot(t, zd)
legend('x', 'y', 'z')