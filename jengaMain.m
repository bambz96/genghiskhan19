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
            obj.robot.enableMotorTorques;
            [x,y,z,theta] = obj.robot.getRobotPose;
            pause(2)
            obj.robot.homePosition();
            
            
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

