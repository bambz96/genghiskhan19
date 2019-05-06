function  Jenga_Tower(x,y,z,theta)

%% Tower Mid point location
% tower_mid.x = 150; %mm
% tower_mid.y = 0; %mm
% tower_mid.z = 0; %mm 
z_upper = 270; %mm
% tower_mid.theta = 0; %deg

Rotation_Matrix = [cosd(theta) sind(theta) 0; -sind(theta) cosd(theta) 0; 0 0 1];

f0_ro_p = [x ; y ; z];
f1_rp_c1 = [-37.5; 37.5; 0];
f1_rp_c2 = [-37.5; -37.5; 0];
f1_rp_c3 = [37.5; -37.5; 0];
f1_rp_c4 = [37.5; 37.5; 0];


c1 = f0_ro_p+ Rotation_Matrix*f1_rp_c1;
c2 = f0_ro_p+ Rotation_Matrix*f1_rp_c2;
c3 = f0_ro_p+ Rotation_Matrix*f1_rp_c3;
c4 = f0_ro_p+ Rotation_Matrix*f1_rp_c4;


c5 = c1 + [0;0;z_upper];
c6 = c2 + [0;0;z_upper];
c7 = c3 + [0;0;z_upper];
c8 = c4 + [0;0;z_upper];


hold on
%Plot Lower Points
plot3(c1(1),c1(2),c1(3),'-r*')
plot3(c2(1),c2(2),c2(3),'-r*')
plot3(c3(1),c3(2),c3(3),'-r*')
plot3(c4(1),c4(2),c4(3),'-r*')


%Plot Upper Points 
plot3(c5(1),c5(2),c5(3),'-r*')
plot3(c6(1),c6(2),c6(3),'-r*')
plot3(c7(1),c7(2),c7(3),'-r*')
plot3(c8(1),c8(2),c8(3),'-r*')

v = [c1'; c2'; c3'; c4';];
f = [1 2 3 4];
patch('Faces',f,'Vertices',v,'FaceColor','blue')


v = [c1'; c2'; c6'; c5';];
f = [1 2 3 4];
patch('Faces',f,'Vertices',v,'FaceColor','blue')

v = [c2'; c3'; c7'; c6';];
f = [1 2 3 4];
patch('Faces',f,'Vertices',v,'FaceColor','blue')

v = [c3'; c4'; c8'; c7';];
f = [1 2 3 4];
patch('Faces',f,'Vertices',v,'FaceColor','blue')

v = [c4'; c1'; c5'; c8';];
f = [1 2 3 4];
patch('Faces',f,'Vertices',v,'FaceColor','blue')

v = [c5'; c6'; c7'; c8';];
f = [1 2 3 4];
patch('Faces',f,'Vertices',v,'FaceColor','blue')


%% 
