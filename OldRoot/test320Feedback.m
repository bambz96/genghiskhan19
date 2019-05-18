% Test XL320 Feedback

tic
figure
hold on
while 1
    [pos, ~] = obj.robot.motorControl.motor5.getPos();
    plot(toc,pos,'r*');
    pause(0.5)
end