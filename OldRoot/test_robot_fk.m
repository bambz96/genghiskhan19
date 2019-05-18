% check it works as expected and respects joint limits set in motorControl

arm = robot();

arm.enableMotorTorques;

% target angles in degrees
q1 = 5;
q2 = 5;
q3 = 5;
q4 = 5;
q5 = 5;

arm.setRobot(q1, q2, q3, q4, q5);

% pause(5)

% arm.motorControl.endConnection();