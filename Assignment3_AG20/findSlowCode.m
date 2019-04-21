jenghis = robot;

x0 = [100,0,200,0];
x_dot_0 = [0,0,0,0];
xf = [150,50,150,0];
x_dot_f = [0,0,0,0];
tf = 5;
dt = 0.1;

% jenghis.differentialKinematics.findJointSpaceVelocities([0;5;10;15;20],[5;5;5;0;0;1]);

[t, q, q_dot] = jenghis.trajectoryPlanning.generateJointTrajectory(x0,x_dot_0,xf,x_dot_f, tf, dt);

