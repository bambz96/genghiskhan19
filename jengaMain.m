classdef jengaMain < handle
    %   JENGAMAIN Main Class for stacking a jenga tower
    %   Robot and tower objects are instantiated here. This class also
    %   controls user input to change tower location
    
    properties
        robot
        tower
        
        jengaX
        jengaY
        jengaTheta
    end
    
    methods
        function obj = jengaMain()
            %STACKJENGA Construct an instance of this class
            obj.robot = robot();
            %obj.tower = jengaTower();
            
            %% Routine
%             obj.robot.disableMotorTorques;
            obj.robot.motorControl.positionMode;
            obj.robot.enableMotorTorques;
            
            pause(2)
            
%             x0 = [250,0,100,0];
%             x_dot_0 = [0,0,0,0];
%             xf = [300,150,250,0];
%             x_dot_f = [0,0,0,0];
            
            x0 = [130,0,180,0];
            x_dot_0 = [0,0,0,0];
            xf = [92,92,280,0];
            x_dot_f = [0,0,0,0];
            
            %% the ol switcheroo
%             x_temp = x0;
%             x0 = xf; 
%             xf = x_temp; 
            
            tf = 5;
            dt = 0.01; 
            
            K1 = 1;
            K2 = 1; 
            K3 = 0.5;
            K4 = 1;
            K5 = 1; 
            
            [t, q, q_dot] = obj.robot.trajectoryPlanning.generateJointTrajectory(x0,x_dot_0,xf,x_dot_f, tf, dt);
         
            obj.robot.setRobotPose(x0(1),x0(2),x0(3),x0(4));
            pause(5); 
            

            disp('Start Trajectory')
            [t_act,q_act] = obj.robot.motorControl.executeTrajectory(t,q,q_dot,[K1 K2 K3 K4 K5]);
            disp('Finished Trajectory')
            
            figure, hold on, grid on
            plot(t,q(:,1),'r',t,q(:,2),'b',t,q(:,3),'g',t,q(:,4),'c',t,q(:,5),'m')
            plot(t_act,q_act(:,1),'r*',t_act,q_act(:,2),'b*',t_act,q_act(:,3),'g*',t_act,q_act(:,4),'c*',t_act,q_act(:,5),'m*')
            legend('q_1 Designed','q_2 Designed','q_3 Designed','q_4 Designed', 'q_5 Designed','q_1 Actual', 'q_2 Actual', 'q_3 Actual', 'q_4 Actual', 'q_5 Actual')
            xlabel('Time (s)')
            ylabel('Joint Angle (deg)')
            str = sprintf('Motor Controller Performance - K1 = %.1f, K2 = %.1f, K3 = %.1f, K4 = %.1f, K5 = %.1f',K1,K2,K3,K4,K5);
            title(str)
            
%             figure
%             for i = 1:5
%                 subplot(1,5,i)
%                 plot(t,q(i))
%             end

            % Test Velocity control
%             obj.robot.motorControl.setVelocities(15,15,15,0,0)
%             pause(1)
%             [v1,v2,v3,v4,v5] = obj.robot.motorControl.getVelocities
%             pause(2)
%             obj.robot.motorControl.setVelocities(-15,-15,-15,0,0)
%             pause(1)
%             [v1,v2,v3,v4,v5] = obj.robot.motorControl.getVelocities
%             pause(2)
%             obj.robot.motorControl.setVelocities(0,0,0,0,0)
            % Test Velocity Readings
%             go = 1; 
%             while go
%                 [v1,v2,v3,v4,v5] = obj.robot.motorControl.getVelocities 
%                 pause(0.5); 
%             end
            
            
            % Test Position Control
%             [x,y,z,theta] = obj.robot.getRobotPose;
%             obj.robot.homePosition();
%             pause(2);
%             obj.robot.setRobotPose(200,0,150,0);
%             pause(2);
%             obj.robot.homePosition();
%             pause(2); 
%             obj.robot.setRobotPose(250,0,400-138,0);
%             pause(2);
%             obj.robot.homePosition();
%             obj.robot.setRobotPose(150,100,400-138,0);
            
            %% End
            pause(5);
            obj.robot.motorControl.endConnection;
        end
        
        function stackTower(x,y,theta)
            %METHOD1 Stack tower at a particular location and orientation
%             obj.jengaX = x;
%             obj.jengaY = y; 
%             obj.jengaTheta = theta; 
        end
        
    end
end

