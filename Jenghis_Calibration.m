 jenghisKhan = robot;

        jenghisKhan.L1      = 200;   % mm        % Origin to joint 1
        jenghisKhan.L2      = 200;	 % mm        % Joint 1+2 to Joint 3
        jenghisKhan.L3      = 200;	 % mm        % Joint 3 to Joint 4
        jenghisKhan.L4      = 100;	 % mm        % Joint 4 to End effector
       
      
jenghisKhan.forwardKinematics = forwardKinematics(jenghisKhan);
plot = jenghisKhan.drawPose(0,0,0,0,0);


%b = [x1; y1; z1; x2; y2; z2; xp; yp; zp;] coordinates of actual grid positions
%that we align the laser with, this is a 3p x 1 matrix

%A = [q1_5x5; q2_5x5; ] matrix of q values which result in position b 

%w = [L1; L2: L3: L4: L5] Link lengths which will be adjusted to fit data
%set


% w = inv(A.'*A)*A.'*b;

