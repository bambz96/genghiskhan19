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
            obj.robot.disableMotorTorques;
            obj.robot.motorControl.velocityMode; 
            obj.robot.enableMotorTorques;
            pause(2)
            
            % Test Velocity control
            obj.robot.motorControl.setVelocities(15,15,15,0,0)
            pause(1)
            [v1,v2,v3,v4,v5] = obj.robot.motorControl.getVelocities
            pause(2)
            obj.robot.motorControl.setVelocities(-15,-15,-15,0,0)
            pause(1)
            [v1,v2,v3,v4,v5] = obj.robot.motorControl.getVelocities
            pause(2)
            obj.robot.motorControl.setVelocities(0,0,0,0,0)
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

