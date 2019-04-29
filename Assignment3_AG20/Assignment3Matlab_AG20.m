%{
    Assignemnt Group 20
    Assignment 3, testing code. 

%}

jenghis = robot;

IK = jenghis.inverseKinematics;

% Sample trajectory
% [x, y, z, theta]
LoadingBay = [37.5; 187.5; -3; 90];

% Sample tower position
Tower = jTower(200, 0, 0);
% Block drop location
FirstBlock = Tower.nextBlock;
EndPoint = FirstBlock.getPosition + [0; 0; 5; 0];

% Via Point 1 
v1 = LoadingBay + [0; 0; 20; 0];
% Via Point 3
v3 = EndPoint + [-10; 0; 5; 0]; 
% Via point 2:
% In this simple instance, via point is located at midpoint of v1 and v2
% Future iterations will allow for adjustment of this via point
v2 = (v1 + v3)/2;

x = [LoadingBay, v1, v2, v3, EndPoint];
t = [0, 0.5, 2.5, 4.5, 5];
ts = 0.1;

Terry = taskTrajectory(x, t, ts, 4);

Joe = jointTrajectory(Terry, jenghis.DoF, IK);

plot(Joe.getTimeseries, Joe.getQ);

legend('q1', 'q2', 'q3', 'q4', 'q5');


