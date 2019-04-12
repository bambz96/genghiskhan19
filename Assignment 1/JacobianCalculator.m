%{ 
Author: Assignment Group 20
William Ellett 586703

Calculation of Robot Jacobian. 
%}
%% Set up
jenghis = robot;

DoF = 5;



%Unit vector z in reference frame
z_hat = [0;0;1];
%Initialze Jacobian
Jacobian = [];

%%  Iteratively Calculates each row of the Jacobian
for joint = 1:DoF
    R = Rotation(jenghis, 0, joint);
    z = R*z_hat;
    P = R*Position(jenghis, joint, 'W');

    Jacobian = [Jacobian, jacobianRevolute(z, P)];   
end
%Final Result
Jacobian = simplify(Jacobian)


%% Helper/Wrapper functions

%Wrapper function for a method called on robot.forwardKinematics
function R = Rotation(robot, frameExpressed, frameOf)
    R = robot.forwardKinematics.getRotation(frameExpressed, frameOf);
end

%Wrapper function for a method called on robot.forwardKinematics
function P = Position(robot, frameExpressed, frameOf)
    P = robot.forwardKinematics.getPosition(frameExpressed, frameOf);
end

%function to calculate jacobian colmumn
function Ji = jacobianRevolute(zi, Pi)
%   Ji = zeros(6, 1); %initialize jacobian
  Ji = cross(zi, Pi);
  Ji= [Ji;zi];
end

