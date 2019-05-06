%{
Assignment Group 20
Cubic spline interpolation using the "spline" function

This script doesn't really do a lot...
%}


x0 = 0; %initial x position
xf = 3; %final x position

t0 = 0; %time zero
tf = 4; %final time

ts = 0.1; %sample time in seconds

v0 = 0; %initial velocity
vf = 0; %final velocity;

x = [x0 -1 xf];

t = [t0 2 tf];

tt = t0:ts:tf; %vector of sampling times

S = spline(t, [v0 x vf]); %cubic spline interpolation

plot(tt, ppval(S, tt)); % Plot the full piecewise spline

hold on; 

plot(tt, polyval(S.coefs(1, :), tt), 'r');
