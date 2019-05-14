clear all, 
% close all
% Assignment 1

  jenghisKhan = robot;

        jenghisKhan.L1      = 112.2541;   % mm        % Origin to joint 1
        jenghisKhan.L2      = 201.4447;	 % mm        % Joint 1+2 to Joint 3
        jenghisKhan.L3      =  201.1605;	 % mm        % Joint 3 to Joint 4
        jenghisKhan.L4      = 15.9061; % mm        % Joint 4 to End effector
% 
%         jenghisKhan.L1      = 200;   % mm        % Origin to joint 1
%         jenghisKhan.L2      = 200;	 % mm        % Joint 1+2 to Joint 3
%         jenghisKhan.L3      = 200;	 % mm        % Joint 3 to Joint 4
%         jenghisKhan.L4      = 100;	 % mm        % Joint 4 to End effector
% %        
      
% jenghisKhan.forwardKinematics = forwardKinematics(jenghisKhan);
% plot = jenghisKhan.drawPose(0,0,0,0,0);

% [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(-65,-52,-23,75,0);
% [x,y,z] = findCoordinates(jenghisKhan.forwardKinematics,90,-45,45,0,0);
% [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(-60,-50,-50,70,0);
% [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(-60,-50,-50,100,0);
% [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(45,0,-60,0,0);
% [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(58.59,-58.50,-30.28,88.87,0);
[x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(-58.72,-53.71,-36.4,90.11,0);


x = x(end)
y = y(end)
z = z(end)
% 
% [q1, q2, q3, q4, q5] = jenghisKhan.inverseKinematics.findQ(87.5,-175,15,0)
% [q1, q2, q3, q4, q5] = jenghisKhan.inverseKinematics.findQ(x, y, z,0)
[q1, q2, q3, q4, q5] = jenghisKhan.inverseKinematics.findQ(87.5,-137.5,15,0)
