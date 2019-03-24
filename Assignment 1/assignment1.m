clear all, 
% close all
% Assignment 1

jenghisKhan = robot;

plot = jenghisKhan.drawPose(0,0,0,0,0);

view(0,0)
hold on
%% Reachable Workspace
for q2 = jenghisKhan.j2_lim(1):5:jenghisKhan.j2_lim(2)
    q2
    for q3 = jenghisKhan.j3_lim(1):5:jenghisKhan.j3_lim(2)
        
           q4 = -(q3+q2);
           [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(0,q2,q3,q4,0);
           
         pause(0.0005);
           
           plot3(x(6),y(6),z(6),'b*'); 
    end
end
 
%% 
 