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
            counter
           q4 = -(q3+q2);
           pause(0.0005);
           
         
%          if q4 >= jenghisKhan.j4_lim(1) && q4 <= jenghisKhan.j4_lim(2)
             if q2 == jenghisKhan.j2_lim(1) || q2 == jenghisKhan.j2_lim(2)
                    [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(0,q2,q3,q4,0);
                    plot3(x(6),y(6),z(6),'-c*');          
             end 

             if q2 > jenghisKhan.j2_lim(1) && q2 < jenghisKhan.j2_lim(2)
                  if q3 == jenghisKhan.j3_lim(1) || q3 == jenghisKhan.j3_lim(2)
                        [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(0,q2,q3,q4,0);
                        plot3(x(6),y(6),z(6),'-c*'); 
                        X_Check(counter) = x(6)
                  end 
             end 
%          end 
         
    end
end
 
%% 
 