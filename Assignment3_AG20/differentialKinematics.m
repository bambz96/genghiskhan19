classdef differentialKinematics

%     Differential Kinematics for robot class, calculates robot jacobian 
%     
%     Usage: class should be initialised in the constructor of the robot to 
%     which these differentaial kinematics are assigned.
%     
%     Currently just calculates jacobian, additional functionality to be
%     added in future release.
    

    
   %%  Properties
    properties 
        Jacobian 
        Frame = 'W';
        
        z_hat = [0;0;1];
    end
    
    %% Methods
    methods
        %Constructor
        function obj = differentialKinematics(robot)

            obj.Jacobian = jacobianCalculator(obj, robot);
        end
        
        
        %Jacobian calculator (most important function)
        function Jacobian = jacobianCalculator(obj, robot)
            Jacobian = [];
            for joint = 1:(robot.DoF)
                R = obj.Rotation(robot, 0, joint);
                z = R*obj.z_hat;
                P = R*obj.Position(robot, joint, obj.Frame);
                
                Jacobian = [Jacobian, obj.jacobianRevolute(z, P)];
                
            end
            %Final Result
            Jacobian = simplify(Jacobian);
        end 
        
        
        %Helper function for calculating jacobian for a single revolute
        %joint
        function Ji = jacobianRevolute(obj, zi, Pi)
            Ji = cross(zi, Pi); %linear velocity
            Ji= [Ji;zi];        %angular velocity
        end
        

        %%  Wrapper functions
        %Wrapper function for a method called on robot.forwardKinematics
        function R = Rotation(obj, robot, frameExpressed, frameOf)
            R = robot.forwardKinematics.getRotation(frameExpressed, frameOf);
        end
        
        %Wrapper function for a method called on robot.forwardKinematics
        function P = Position(obj, robot, frameExpressed, frameOf)
            P = robot.forwardKinematics.getPosition(frameExpressed, frameOf);
        end
        
    end

end   