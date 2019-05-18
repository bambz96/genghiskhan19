%{
    Project Group 10
    Quick little hack up of a script for testing some stuff with the 
    controller board
    
%}

% The test trajectory here is taken from Assignment 3 code, and represents
% the trajectory to place the first block



%% Test Parameters
ts = 0.1;               % sample time (s)

dropH = 5;              % block drop height (mm)
totalTime = 5;          % time for full tajectory
liftTime = 0.5;         % time to first via point
approachTime = 0.5;     % time to second via point



%% Set up robot
jenghis = robot;

IK = jenghis.inverseKinematics;
DK = jenghis.differentialKinematics;

%% Sample trajectory
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


%% Make trajectory
x = [LoadingBay, v1, v2, v3, EndPoint];
t = [0, liftTime, totalTime/2, (totalTime-approachTime), totalTime];


Terry = taskTrajectory(x, t, ts, 4);

%% Write trajectory to CSV

csvwrite("trajectory.csv", Terry.getPosition');





