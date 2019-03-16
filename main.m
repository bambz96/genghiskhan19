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

motor1 = XL320(motor1_ID, port_num); 
motor1.torqueEnable;

%% Routine

dxl_present_position = motor1.getPos;
fprintf('PresPos:%03d\n', typecast(uint32(dxl_present_position), 'int32'));
motor1.setPos(500);
motor1.getPos; 
fprintf('PresPos:%03d\n', typecast(uint32(dxl_present_position), 'int32'));
pause(5);
motor1.setPos(100);
motor1.getPos;
fprintf('PresPos:%03d\n', typecast(uint32(dxl_present_position), 'int32'));

%% Clean Up

motor1.torqueDisable;

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;







