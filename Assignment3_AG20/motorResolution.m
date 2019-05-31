jenghis = robot;

%% Find ideal angles
x = 200;
y = 0;
z = 10;
theta = 0; 

[q1t, q2t, q3t, q4t, q5t] = jenghis.inverseKinematics.findQ(x,y,z,theta);

% [x,y,z] = jenghis.findCoordinates(q1t,q2t,q3t,q4t,q5t);

%% Script to work out impact of resolution
resolution = 0.01:0.01:0.5;
error = zeros(1,length(resolution));
threshold = 2;

for i = 1:length(resolution)
    [xc,yc,zc] = jenghis.forwardKinematics.findCoordinates(q1t+resolution(i),q2t+resolution(i),q3t+resolution(i),q4t+resolution(i),q5t+resolution(i));
    error(i) = sqrt((xc(end)-x)^2+(yc(end)-y)^2+(zc(end)-z)^2);
end

plot(resolution, error, resolution, threshold*ones(1,length(resolution))); 

xlabel('Encoder resolution (degrees)')
ylabel('Absolute position error (mm)')

title('Impact of encoder resolution on robot performance') 

legend('Calculated Error','Error Threshold')


