clear all, 
% close all
% Assignment 1

jenghisKhan = robot;

plot = jenghisKhan.drawPose(0,0,0,0,0);

view(0,0)
hold on
%% Reachable Workspace

counter = 1
for q2 = jenghisKhan.j2_lim(1):5:jenghisKhan.j2_lim(2)
    q2
    for q3 = jenghisKhan.j3_lim(1):5:jenghisKhan.j3_lim(2)
        
           q4 = -(q3+q2);
          
           
           if q2 == jenghisKhan.j2_lim(1)  
                [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(0,q2,q3,q4,0);
                Coords(counter,1) = x(6); 
                Coords(counter,2) = y(6); 
                Coords(counter,3) = z(6);
                counter = counter+1;
           end    
           if q2 > jenghisKhan.j2_lim(1) 
               if q3 == jenghisKhan.j2_lim(1)
                        [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(0,q2,q3,q4,0);
                    Coords(counter,1) = x(6); 
                    Coords(counter,2) = y(6); 
                    Coords(counter,3) = z(6);    
                    counter = counter+1;
               end 
               if q3 == jenghisKhan.j2_lim(2)
                     [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(0,q2,q3,q4,0);
                    Coords(counter,1) = x(6); 
                    Coords(counter,2) = y(6); 
                    Coords(counter,3) = z(6);    
                    counter = counter+1;
               end    
           end
    end
 end
 
%% 
 i = 0;
% for i = 1:counter-1
%     plot3(Coords(i,1),Coords(i,2),Coords(i,3),'k*');
% end 


    plot3(Coords(:,1),Coords(:,2),Coords(:,3),'k*');


