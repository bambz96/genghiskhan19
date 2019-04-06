clc, clear all, close all

initLib; % Load Libraries

%% Open Port and set Baud Rate
BAUDRATE = 57600;
DEVICENAME = 'COM3';

initPort;
initBAUD; 

% Initialize PacketHandler Structs
packetHandler();

%% Motor Initialisation
motor1_ID = 1; 

motor1 = XL320(motor1_ID, port_num); 
motor1.torqueEnable;

%% Routine

speed1 = 800;
speed2 = 500;
speed3 = 1523;

targetSpeed = [];
measuredSpeed = [];
time = [];

motor1.torqueDisable;
motor1.wheelMode;
motor1.torqueEnable;

motor1.setSpeed(speed1);
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







