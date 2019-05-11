
%%just outlines the steps to calculate given Jacobian input
syms q1 J

jenghis = robot();
J = jenghis.differentialKinematics.Jacobian;
R01 = [cos(q1) -sin(q1) 0 ; sin(q1) cos(q1) 0; 0 0 1];
J_1 = [R01 zeros(3); zeros(3) R01] * J;
J_1(4,:) = [];
invJdeg = inv(J_1);
invJdeg = simplify(invJdeg, 'Steps', 300);
%%invJrad and invJdeg should be the same apart from a pi/180 in
%%angles
J_1rad = subs(J_1, pi/180, 1);
invJrad = inv(J_1rad);
invJrad = simplify(invJrad, 'Steps', 300);
