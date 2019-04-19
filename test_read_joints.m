% read joint angles from absolute encoders, do not enable motor torque

arm = robot();

% simple GUI used to interrupt loop and view joint angles
ButtonHandle = uicontrol('Style', 'PushButton', 'String', 'Stop loop', 'Callback', 'delete(gcbf)');
textHandle = uicontrol('Style', 'text', 'String', 'q1 q2 q3 q4 q5', 'Position', [150, 150, 100, 15]);

while 1
    if ~ishandle(ButtonHandle)
        disp('Loop stopped by user');
        break;
    end
%     q1 = 30;
    q2 = 40;
    q3 = 50;
    q4 = 60;
    q5 = 70;
    q1 = arm.motorControl.motor1.getPos;
%     q2 = arm.motorControl.motor2.getPos();
%     q3 = arm.motorControl.motor3.getPos();
%     q4 = arm.motorControl.motor4.getPos();
%     q5 = arm.motorControl.motor5.getPos();
    textHandle.String = sprintf('%d %d %d %d %d', q1, q2, q3, q4, q5);
    pause(0.1);
end

arm.motorControl.endConnection();

disp('Connection ended.')