close all
%% define constraints
p0 = [0.4 0 0.05];
pf = [0.2 0.3 0.3];
v0 = [0 0 0];
vf = [0 0 0];
%% time
tv = 1.2; % time to via point
tf = 2; % time to reach end
t = 0:0.01:tf;
%% find paths
[x, xd, xdd] = cubic_via(p0(1), v0(1), pf(1), vf(1), via(1), tv, tf);
[y, yd, ydd] = cubic_via(p0(2), v0(2), pf(2), vf(2), via(2), tv, tf);
[z, zd, zdd] = cubic_via(p0(3), v0(3), pf(3), vf(3), via(3), tv, tf);
%% plot
subplot(221)
hold on
title('Position')
ylabel('Position (m)')
xlabel('Time (s)')
plot(t, x)
plot(t, y)
plot(t, z)
legend('x', 'y', 'z')

subplot(223)
hold on
title('Velocity')
ylabel('Velocity (m/s)')
xlabel('Time (s)')
plot(t, xd)
plot(t, yd)
plot(t, zd)
legend('x', 'y', 'z')

subplot(2,2, [2 4])
hold on
grid on
title('Path')
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
plot3(x, y, z)
scatter3([p0(1) pf(1) via(1)], [p0(2) pf(2) via(2)], [p0(3) pf(3) via(3)])
view(30, 15);