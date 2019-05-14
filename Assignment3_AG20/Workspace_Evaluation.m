clear all, 
% close all
% Assignment 1

  jenghisKhan = robot;

        jenghisKhan.L1      = 200;   % mm        % Origin to joint 1
        jenghisKhan.L2      = 200;	 % mm        % Joint 1+2 to Joint 3
        jenghisKhan.L3      = 200;	 % mm        % Joint 3 to Joint 4
        jenghisKhan.L4      = 100;	 % mm        % Joint 4 to End effector
       
      
jenghisKhan.forwardKinematics = forwardKinematics(jenghisKhan);
plot = jenghisKhan.drawPose(0,-90,0,90,0);

view(0,0)
hold on
counter = 1;
counter2 = 1;
k = 1;
%% Reachable Workspace
for q2 = jenghisKhan.j2_lim(1):5:jenghisKhan.j2_lim(2)
    q2;
    for q3 = jenghisKhan.j3_lim(1):5:jenghisKhan.j3_lim(2)
        
           q4 = -(q3+q2);
           pause(0.0005);
           
             if q2 == jenghisKhan.j2_lim(1) || q2 == jenghisKhan.j2_lim(2)
                    [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(0,q2,q3,q4,0);
                    if jenghisKhan.j4_lim(1) <= q4 <= jenghisKhan.j4_lim(2)
                            plot3(x(5),y(5),z(5),'-b*'); 
                    end 
                    if q4 < jenghisKhan.j4_lim(1)  
                            plot3(x(5),y(5),z(5),'-r*'); 
                    end
                    
                    %find min x radius of jenga build zone 
                    if q2 == jenghisKhan.j2_lim(2)       
                        x_min(counter) = x(5);
                        counter = counter+1;
                    end
    
             end 

             if q2 >= jenghisKhan.j2_lim(1) && q2 <= jenghisKhan.j2_lim(2)
                  if q3 == jenghisKhan.j3_lim(1) || q3 == jenghisKhan.j3_lim(2)
                       [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(0,q2,q3,q4,0);
                       if jenghisKhan.j4_lim(1) <= q4 <= jenghisKhan.j4_lim(2)
                            plot3(x(5),y(5),z(5),'-b*'); 
                    end 
                    if q4 < jenghisKhan.j4_lim(1)  
                            plot3(x(5),y(5),z(5),'-r*'); 
                    end
                        
                       %find max x radius of jenga build zone 
                        if q3 == jenghisKhan.j3_lim(2)
                            if z(5) > 275 && z(5) <315 && x(5) > 70
                            x_max(k) = x(5);
                            k = k+1;
                            end 
                        end 
                        % Find other min x radius of jenga build zone, need
                        % to compare to previous
                         if q3 == jenghisKhan.j3_lim(1)
                             x_min_other(counter2) = x(5);
                             counter2 = counter2+1;
                           
                         end 


                  end 
             end 
     end 
end    
   
    

 if max(x_min_other) > max(x_min)
    x_min = x_min_other;
 end 
    

%% Plot Jenga Tower 

Minimum_X = max(x_min);
Maximum_X = max(x_max);

x_range = Maximum_X - Minimum_X;

if Maximum_X - 70 > Minimum_X    
Jenga_Tower(Minimum_X+37.5,0,0,0)

Jenga_Tower(Maximum_X-37.5,0,0,0)
else
    fprintf('error - buildspace too small \n')
    
end 
 