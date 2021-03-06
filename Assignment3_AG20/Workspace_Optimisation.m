
clear all
pause(1)
jenghisKhan = robot;

% plot = jenghisKhan.drawPose(0,0,0,0,0);

% view(0,0)
% hold on

        L1_min      = 150;   % mm        % Origin to joint 1
        L1_max      = 250;
        L1_step     = 50;
        
        L2_min      = 150;	 % mm        % Joint 1+2 to Joint 3
        L2_max      = 250;
        L2_step     = 50;
        
        L3_min      = 150;	 % mm        % Joint 3 to Joint 4
        L3_max      = 250;
        L3_step     = 50;
        
        L4_min      = 50;	 % mm        % Joint 4 to Joint 5
        L4_max      = 70;
        L4_step     = 10;
        
        LE_min      = 50;	 % mm        % Joint 5 to End Effector
        LE_max      = 90;
        LE_step     = 20;
        
        Build_X_Min = 200

for L1 = L1_min:L1_step:L1_max
jenghisKhan.L1 = L1;
1
    for L2 = L2_min:L2_step:L2_max
        jenghisKhan.L2 = L2;
        2
        for L3 = L3_min:L1_step:L3_max
            jenghisKhan.L3 = L3;
            3
            for L4 = L4_min:L4_step:L4_max
                jenghisKhan.L4 = L4;
                4
                for LE = LE_min:LE_step:LE_max
                    jenghisKhan.LE = LE;
                    5
                    assignment1_3;
    
                    x_range = x_max - max(x_array);
                    counter3 = 1;
                    if x_range > Build_X_Min
                         L_Tot = jenghisKhan.L1+2*jenghisKhan.L2+jenghisKhan.L3;   
                         viable(counter3,1) = jenghisKhan.L1;
                         viable(counter3,2) = jenghisKhan.L2;
                         viable(counter3,3) = jenghisKhan.L3;
                         viable(counter3,4) = jenghisKhan.L4;
                         viable(counter3,5) = jenghisKhan.LE;
                         viable(counter3,6) = L_Tot;
                         
                    end     

            end     
    
    
    
        end     
    
    
    
    end     
    
    
    
    end
end 

%% test
    
  
%     
%     for i = 1:length(viable)
%         length_total = viable(i,6);
%         
%         if length_total = Min(viable(:,6);
%             jenghisKhan.L1
%             jenghisKhan.L2
%             jenghisKhan.L3
%             jenghisKhan.L4
%             jenghisKhan.LE
%         end 
%     end
%     
    
    
    
    
     
     