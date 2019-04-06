clc, clear all, close all

initLib; % Load Libraries

%% Open Port and set Baud Rate
BAUDRATE = 1000000;
DEVICENAME = 'COM3';

initPort;
initBAUD; 

% Initialize PacketHandler Structs
packetHandler();

%% Motor Initialisation
motor1_ID = 1; 

motor1 = XL430(motor1_ID, port_num); 
motor1.torqueEnable;

%% Routine

speed1 = 30;
speed2 = 500;
speed3 = 1523;

targetSpeed = [];
measuredSpeed = [];
time = [];

motor1.positionMode; 
motor1.setPos(180);
pause(2);
motor1.getPos
motor1.setPos(135);
pause(2);
motor1.getPos
motor1.setPos(90);
pause(2); 
motor1.getPos
motor1.setPos(180);
pause(2); 
motor1.getPos

% motor1.torqueDisable;
% motor1.velocityMode;
% motor1.torqueEnable;
% 
% motor1.setVelocity(speed1);
% motor1.getVelocity
% pause(3)
% motor1.getVelocity
% pause(3)
% motor1.getVelocity
% pause(10)
% motor1.getVelocity
% tic
% while toc < 15
%     targetSpeed = [targetSpeed speed1];
%     measuredSpeed = [measuredSpeed motor1.getSpeed];
%     time = [time toc];
% end
% motor1.setSpeed(speed2);
% while toc < 10
%     targetSpeed = [targetSpeed speed2];
%     measuredSpeed = [measuredSpeed motor1.getSpeed];
%     time = [time toc];
% end
% motor1.setSpeed(speed3);
% while toc < 15
%     targetSpeed = [targetSpeed speed3];
%     measuredSpeed = [measuredSpeed motor1.getSpeed];
%     time = [time toc];
% end


    

% dxl_present_speed = motor1.getSpeed;
% fprintf('PresSpeed:%03d\n', typecast(uint32(dxl_present_speed), 'int32'));
% motor1.setSpeed(1000);
% dxl_present_speed = motor1.getSpeed;
% fprintf('PresSpeed:%03d\n', typecast(uint32(dxl_present_speed), 'int32'));
% pause(2);
% motor1.setSpeed(0);
% pause(2);
% motor1.setSpeed(2000);
% pause(2);
% motor1.setSpeed(0);
% dxl_present_speed = motor1.getSpeed;
% fprintf('PresSpeed:%03d\n', typecast(uint32(dxl_present_speed), 'int32'));



%% Clean Up

motor1.torqueDisable;

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);







