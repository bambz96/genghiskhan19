%{ 
Author: Assignment Group 20
William Ellett 586703

Assignment2...
%}
%% Set up
jenghis = robot;

DoF = 5;



%Unit vector z in reference frame
z_hat = [0;0;1];

%% Actual Calculation
% Calculates each row of the Jacobian and places it in array
joint = 5;

R = Rotation(jenghis, 0, joint);
z = R*z_hat
P = R*Position(jenghis, joint, 'W');
P = simplify(P)

Jacobian_i = jacobianRevolute(z, P);
Jacobian_i = simplify(Jacobian_i)  

%% Wrappper/Helper functions.

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