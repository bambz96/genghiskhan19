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
% motor1.torqueEnable;

%% Routine
motor1.torqueDisable;
motor1.jointMode;
motor1.torqueEnable;

dxl_present_position = motor1.getPos;
fprintf('PresPos:%03d\n', typecast(uint32(dxl_present_position), 'int32'));
motor1.setPos(500);
dxl_present_position = motor1.getPos;
fprintf('PresPos:%03d\n', typecast(uint32(dxl_present_position), 'int32'));
pause(2);
motor1.setPos(700);
dxl_present_position = motor1.getPos;
fprintf('PresPos:%03d\n', typecast(uint32(dxl_present_position), 'int32'));
pause(2);

motor1.torqueDisable;
motor1.wheelMode;
motor1.torqueEnable;

dxl_present_speed = motor1.getSpeed;
fprintf('PresSpeed:%03d\n', typecast(uint32(dxl_present_speed), 'int32'));
motor1.setSpeed(1000);
dxl_present_speed = motor1.getSpeed;
fprintf('PresSpeed:%03d\n', typecast(uint32(dxl_present_speed), 'int32'));
pause(2);
motor1.setSpeed(0);
pause(2);
motor1.setSpeed(2000);
pause(2);
motor1.setSpeed(0);
dxl_present_speed = motor1.getSpeed;
fprintf('PresSpeed:%03d\n', typecast(uint32(dxl_present_speed), 'int32'));



%% Clean Up

motor1.torqueDisable;

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;







