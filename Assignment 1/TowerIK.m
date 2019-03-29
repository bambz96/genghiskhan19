%{
 Tests inerse Kinematics on veriticies of tower
 Quick little hack job...
%}
%% Set location of Tower
x_t = 200;
y_t = 0;
z_t = 0;
theta = 0;

z_upper = 270;

ZRotation_Matrix = [cosd(theta) sind(theta) 0; -sind(theta) cosd(theta) 0; 0 0 1];

f0_ro_p = [x_t ; y_t ; z_t];
f1_rp_c1 = [-37.5; 37.5; 0];
f1_rp_c2 = [-37.5; -37.5; 0];
f1_rp_c3 = [37.5; -37.5; 0];
f1_rp_c4 = [37.5; 37.5; 0];


c1 = f0_ro_p+ ZRotation_Matrix*f1_rp_c1;
c2 = f0_ro_p+ ZRotation_Matrix*f1_rp_c2;
c3 = f0_ro_p+ ZRotation_Matrix*f1_rp_c3;
c4 = f0_ro_p+ ZRotation_Matrix*f1_rp_c4;


c5 = c1 + [0;0;z_upper];
c6 = c2 + [0;0;z_upper];
c7 = c3 + [0;0;z_upper];
c8 = c4 + [0;0;z_upper];

verticies = [c1, c2, c3, c4, c5, c6, c7, c8];

%% Make and test robot

Jenghis = robot;

for v = verticies
    [q1,q2,q3,q4,q5] = Jenghis.inverseKinematics.findQ(v(1), v(2), v(3), 0);
    Jenghis.drawPose(q1,q2,q3,q4,q5);
end