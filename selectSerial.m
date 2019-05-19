function serialOut = selectSerial()
%% select serial port
disp('Available COM ports:')
ports = seriallist();
options = string(zeros(1, length(ports)));
for i = 1:length(ports)
    options(i) = string(i) +':' + ports(i);
end
disp(join(options))
% i = input('Select option:');
i = 1; 
serialOut = serial(ports(i),'BAUD',57600);
%% pause to make sure it's opened
fopen(serialOut);
pause(1);
disp('Connected to '+ports(i))
end