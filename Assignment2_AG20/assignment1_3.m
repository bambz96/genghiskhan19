% clear all, 
% close all
% Assignment 1

jenghisKhan = robot;

% plot = jenghisKhan.drawPose(0,0,0,0,0);

% view(0,0)
% hold on
counter = 1;
counter2 = 1;
%% Reachable Workspace
for q2 = jenghisKhan.j2_lim(1):5:jenghisKhan.j2_lim(2)
    q2;
    for q3 = jenghisKhan.j3_lim(1):5:jenghisKhan.j3_lim(2)
        
           q4 = -(q3+q2);
%            pause(0.0005);
           
         
         if q4 >= jenghisKhan.j4_lim(1) && q4 <= jenghisKhan.j4_lim(2)
             if q2 == jenghisKhan.j2_lim(1) || q2 == jenghisKhan.j2_lim(2)
                    [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(0,q2,q3,q4,0);
%                     plot3(x(6),y(6),z(6),'-r*'); 
                    
                    %find min x radius of jenga build zone 
                    if q2 == jenghisKhan.j2_lim(2)       
                        x_array_min(counter) = x(6);
                        counter = counter+1;
                    end
                    
             end 

             if q2 > jenghisKhan.j2_lim(1) && q2 < jenghisKhan.j2_lim(2)
                  if q3 == jenghisKhan.j3_lim(1) || q3 == jenghisKhan.j3_lim(2)
                       [x,y,z] = jenghisKhan.forwardKinematics.findCoordinates(0,q2,q3,q4,0);
%                        plot3(x(6),y(6),z(6),'-r*'); 
                        
                       %find max x radius of jenga build zone 
                        if q3 == jenghisKhan.j3_lim(2)
                            if z(6) > 270 && z(6) <310
                            x_max = x(6);
                            end 
                        end 
                        % Find other min x radius of jenga build zone, need
                        % to compare to previous
                         if q3 == jenghisKhan.j3_lim(1)
                             x_array_min_other(counter2) = x(6);
                             counter2 = counter2+1;
                           
                        end 


                  end 
             end 
         end 
         
    end
end

 if max(x_array_min_other) > max(x_array_min)
    x_array = x_array_min_other;
 end 
    

%% Plot Jenga Tower 

% Jenga_Tower(max(x_array)+37.5,0,0,0)

% Jenga_Tower(x_max-37.5,0,0,0)
 