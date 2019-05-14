%{
    Assignemnt Group 20
    Assignment 3, testing code. 

    This Script creates an example trajectory using the loading bay
    location, and the first block location, as generated by the Tower class
    There are 3 via points included in the Example trajectory: 
    - v1: above the pick up point
    - v3: determining the tower approach
    - v2: movable point, can be tuned to improve robot performance. Or to
    avoid collision if required. 
    
    These points are used to create a task space trajectory, which is in 
    turn converted into a joint space trajectory

%}

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

% Convert to joint trajectory
Joe = jointTrajectory(Terry, jenghis.DoF, IK, DK);

%% Plots
figure(1)
plot(Terry.getTimeseries, Terry.getPosition);
title('Task Space Trajectory');
legend('x', 'y', 'z', 'theta');

figure(2)
plot(Joe.getTimeseries, Joe.getQ);
title('Joint Space Trajectory');
legend('q1', 'q2', 'q3', 'q4', 'q5');


figure(3) 
plot(Joe.getTimeseries, Joe.getQ_dot);
title('Joint Space Velocity');
legend('q1', 'q2', 'q3', 'q4', 'q5');


