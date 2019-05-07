function matrix = DH(a, alpha, z, theta)
% returns 4x4 affine transformation matrix for the given DH parameters
matrix = Dx(a)*Rx(alpha)*Dz(z)*Rz(theta);
end

function matrix = Dx(a)
% returns 4x4 matrix for translation in x by a
R = eye(3);
r = [a 0 0]';
matrix = [R r; 0 0 0 1];
end

function matrix = Dz(z)
% returns 4x4 matrix for translation in z by z
R = eye(3);
r = [0 0 z]';
matrix = [R r; 0 0 0 1];
end

function matrix = Rx(alpha)
% returns 4x4 matrix for rotation in x by alpha
R = rot('x', alpha);
r = [0 0 0]';
matrix = [R r; 0 0 0 1];
end

function matrix = Rz(theta)
% returns 4x4 matrix for rotation in z by theta
R = rot('z', theta);
r = [0 0 0]';
matrix = [R r; 0 0 0 1];
end

function R = rot(axis, angle)
% returns the 3x3 rotation matrix about the given axis and angle (degrees)
angle = deg2rad(angle);
if strcmp(axis, 'x')
    R = [1 0 0; 0 cos(angle) -sin(angle); 0 sin(angle) cos(angle)];
elseif strcmp(axis, 'y')
    R = [cos(angle) 0 sin(angle); 0 1 0; -sin(angle) 0 cos(angle)];
elseif strcmp(axis, 'z')  
    R = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
end
end