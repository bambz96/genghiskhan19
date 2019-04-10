%{ 
Author: Assignment Group 20
William Ellett 586703

Calculation of Robot Jacobian. 
%}

%Unit vector z in reference frame
z_hat = [0;0;1];

z0_1 = 








function Ji = jacobianRevolute(zi, Pi)
  Ji = zeros(6, 1); %initialize jacobian
  Ji(1:3) = cross(zi, Pi);
  Ji(3:6) = zi; 
end