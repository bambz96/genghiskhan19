classdef roboDraw
    %ROBODRAW Handles plotting the pose of the robot
    %   Detailed explanation goes here
    
    properties
        %% Component Coordinates
        % Origin
        x_Origin
        y_Origin
        z_Origin
        
        % Joint 1 Coord
        x_j1
        y_j1
        z_j1
        
        % Joint 2 Coord
        x_j2
        y_j2
        z_j2
        
        % Joint 3 Coord
        x_j3
        y_j3
        z_j3
        
        % Joint 4 Coord
        x_j4
        y_j4
        z_j4
        
        % Joint 5 Coord
        x_j5
        y_j5
        z_j5
        
        % End Effector Coord
        x_jE
        y_jE
        z_jE
        
        % Joint Sphere radius
        sphereRad = 8; % mm
    end
    
    methods
        function obj = roboDraw(x,y,z)
            obj.x_j1 = x(1);
            obj.y_j1 = y(1);
            obj.z_j1 = z(1);
            
            obj.x_j2 = x(2);
            obj.y_j2 = y(2);
            obj.z_j2 = z(2);
            
            obj.x_j3 = x(3);
            obj.y_j3 = y(3);
            obj.z_j3 = z(3);
            
            obj.x_j4 = x(4);
            obj.y_j4 = y(4);
            obj.z_j4 = z(4);
            
            obj.x_j5 = x(5);
            obj.y_j5 = y(5);
            obj.z_j5 = z(5);
            
            obj.x_jE = x(6);
            obj.y_jE = y(6);
            obj.z_jE = z(6);
            
%             figure
            % Plot parameters
            hold on
            grid on
            daspect([1 1 1])
            view(0,0)
            obj.plotJoints(obj)
            obj.plotLinks(obj)
            
            
        end
    end
    
    methods(Static)
        
        function plotJoints(obj)
            sphereRad = obj.sphereRad; 
            [xSphere, ySphere, zSphere] = sphere;
            surf(xSphere*sphereRad, ySphere*sphereRad, zSphere*sphereRad);
            surf(xSphere*sphereRad+obj.x_j1, ySphere*sphereRad+obj.y_j1, zSphere*sphereRad+obj.z_j1);
            surf(xSphere*sphereRad+obj.x_j2, ySphere*sphereRad+obj.y_j2, zSphere*sphereRad+obj.z_j2);
            surf(xSphere*sphereRad+obj.x_j3, ySphere*sphereRad+obj.y_j3, zSphere*sphereRad+obj.z_j3);
            surf(xSphere*sphereRad+obj.x_j4, ySphere*sphereRad+obj.y_j4, zSphere*sphereRad+obj.z_j4);
            surf(xSphere*sphereRad+obj.x_j5, ySphere*sphereRad+obj.y_j5, zSphere*sphereRad+obj.z_j5);
            surf(xSphere*sphereRad+obj.x_jE, ySphere*sphereRad+obj.y_jE, zSphere*sphereRad+obj.z_jE);
        end
        
        function plotLinks(obj)
            % Link 1
            plot3([0 obj.x_j1],[0 obj.y_j1],[0 obj.z_j1],'k','LineWidth',2)
            
            % Link 2
            plot3([obj.x_j1 obj.x_j2],[obj.y_j1 obj.y_j2],[obj.z_j1 obj.z_j2],'k','LineWidth',2)
            
            % Link 3
            plot3([obj.x_j2 obj.x_j3],[obj.y_j2 obj.y_j3],[obj.z_j2 obj.z_j3],'k','LineWidth',2)
            
            % Link 4
            plot3([obj.x_j3 obj.x_j4],[obj.y_j3 obj.y_j4],[obj.z_j3 obj.z_j4],'k','LineWidth',2)
            
            % Link 4
           plot3([obj.x_j4 obj.x_j5],[obj.y_j4 obj.y_j5],[obj.z_j4 obj.z_j5],'k','LineWidth',2)
           
           % Link 5
           plot3([obj.x_j5 obj.x_jE],[obj.y_j5 obj.y_jE],[obj.z_j5 obj.z_jE],'k','LineWidth',2)
           
           % End Effector
%            x_jE_a 
%            x_jE_b
%            
%            y_jE_a
%            y_jE_b
%            
%            z_jE_a = z_jE;
%            z_jE_b = z_jE;
            
            
        end
    end
    
end

