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
            
            obj.testBot();
        end
        
        function stackTower(x,y,theta)
            %METHOD1 Stack tower at a particular location and orientation
%             obj.jengaX = x;
%             obj.jengaY = y; 
%             obj.jengaTheta = theta; 
        end
        
        function testBot(obj)
            obj.robot.setRobot(180,180,180,180,180)
            pause(2); 
            obj.robot.setRobot(225,225,225,225,225);
            pause(2);
            obj.robot.setRobot(135,135,135,135,135);
            pause(2);
            obj.robot.setRobot(180,180,180,180,180);
            pause(2);
            obj.robot.motorControl.endConnection;
        end
    end
end

