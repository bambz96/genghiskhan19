clear all


jenghisKhan = robot;
%% Set Up A Matrix using R_xyz, values below are copied from R_xyz as components after each L1-L4
syms q1 q2 q3 q4
A11 = 0;
A12 = cos((pi*q1)/180)*cos((pi*(q2 + 90))/180);
A13 = (cos((pi*q1)/180)*cos((pi*(q2 + 90))/180)*cos((pi*(q3 - 90))/180) - cos((pi*q1)/180)*sin((pi*(q2 + 90))/180)*sin((pi*(q3 - 90))/180));
A14 = (cos((pi*q4)/180)*(cos((pi*q1)/180)*cos((pi*(q2 + 90))/180)*sin((pi*(q3 - 90))/180) + cos((pi*q1)/180)*cos((pi*(q3 - 90))/180)*sin((pi*(q2 + 90))/180)) + sin((pi*q4)/180)*(cos((pi*q1)/180)*cos((pi*(q2 + 90))/180)*cos((pi*(q3 - 90))/180) - cos((pi*q1)/180)*sin((pi*(q2 + 90))/180)*sin((pi*(q3 - 90))/180)));
A21 = 0;
A22 = sin((pi*q1)/180)*cos((pi*(q2 + 90))/180);
A23 = (sin((pi*q1)/180)*cos((pi*(q2 + 90))/180)*cos((pi*(q3 - 90))/180) - sin((pi*q1)/180)*sin((pi*(q2 + 90))/180)*sin((pi*(q3 - 90))/180));
A24 = (cos((pi*q4)/180)*(sin((pi*q1)/180)*cos((pi*(q2 + 90))/180)*sin((pi*(q3 - 90))/180) + sin((pi*q1)/180)*cos((pi*(q3 - 90))/180)*sin((pi*(q2 + 90))/180)) + sin((pi*q4)/180)*(sin((pi*q1)/180)*cos((pi*(q2 + 90))/180)*cos((pi*(q3 - 90))/180) - sin((pi*q1)/180)*sin((pi*(q2 + 90))/180)*sin((pi*(q3 - 90))/180)));
A31 = 1;
A32 = sin((pi*(q2 + 90))/180);
A33 = (cos((pi*(q2 + 90))/180)*sin((pi*(q3 - 90))/180) + cos((pi*(q3 - 90))/180)*sin((pi*(q2 + 90))/180));
A34 = -(cos((pi*q4)/180)*(cos((pi*(q2 + 90))/180)*cos((pi*(q3 - 90))/180) - sin((pi*(q2 + 90))/180)*sin((pi*(q3 - 90))/180)) - sin((pi*q4)/180)*(cos((pi*(q2 + 90))/180)*sin((pi*(q3 - 90))/180) + cos((pi*(q3 - 90))/180)*sin((pi*(q2 + 90))/180)));

A_single = [A11 A12 A13 A14; A21 A22 A23 A24; A31 A32 A33 A34];


%% Set up Least Squares Regression


Calibration_Data = csvread('Calibration_DataActual5.csv',1,0); 
max = size(Calibration_Data);    
i = 0;
for j = 1:max(1)
    q1 = Calibration_Data(j,6);
    q2 = Calibration_Data(j,9);
    q3 = Calibration_Data(j,12);
    q4 = Calibration_Data(j,15);
        
        for k = 1:3
            i=i+1;
            A(i,1) = double(subs(A_single(k,1)));
            A(i,2) = double(subs(A_single(k,2)));
            A(i,3) = double(subs(A_single(k,3)));
            A(i,4) = double(subs(A_single(k,4)));
            b(i,1) = Calibration_Data(j,k);
        end  
end 

w = (A.'*A)\A.'*b;

L1 = w(1)
L2 = w(2)
L3 = w(3)
L4 = w(4)

%% Set up Least Squares Regression - Iteration
% figure
% hold on
% 
% Calibration_Data = csvread('Calibration_Data_Test.csv',1,0); 
% 
% max = size(Calibration_Data);
% for counter = 5:max(1)
%     
% i = 0;
% for j = 1:counter
%     q1 = Calibration_Data(j,5);
%     q2 = Calibration_Data(j,7);
%     q3 = Calibration_Data(j,9);
%     q4 = Calibration_Data(j,11);
%         
%         for k = 1:3
%             i=i+1;
%             A(i,1) = double(subs(A_single(k,1)));
%             A(i,2) = double(subs(A_single(k,2)));
%             A(i,3) = double(subs(A_single(k,3)));
%             A(i,4) = double(subs(A_single(k,4)));
%             b(i,1) = Calibration_Data(j,k);
%         end  
% end 
% 
% w = (A.'*A)\A.'*b;
% 
% L1 = w(1);
% L2 = w(2);
% L3 = w(3);
% L4 = w(4);
% 
% 
% plot(counter,L1,'k*',counter,L2,'b*',counter,L3,'r*',counter,L4,'g*')
% pause(0.02);
% end 

%% Checking that the values for A_single are correct
% % 
% %Find pose from 0 to E
% PoseOE = jenghisKhan.forwardKinematics.T_0E;
% 
% %Only use the r vectors for x y and z (last column, top 3 rows)
% R_xyz = [PoseOE(1,4); PoseOE(2,4); PoseOE(3,4)]
% 
% L_Matrix = [jenghisKhan.L1; jenghisKhan.L2; jenghisKhan.L3; 
% jenghisKhan.L4]; %Set up matrix of link lengths
% 
% R_xyz1 = A_single*L_Matrix;
% 
% q1 = 0;
% q2 = 0;
% q3 = 0;
% q4 = 0;
% L1 = 200;
% L2 = 200;
% L3 = 200;
% L4 = 100;
% Coeff_Initial = double(subs(R_xyz));
% Coeff_New = double(subs(R_xyz1));
% 
% %Coeff_Initial should be the same as Coeff_New



