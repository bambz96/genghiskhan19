classdef motorControl < handle
    %MOTORCONTROL Handles motor control
    %   Initialisation of motors, and sending and receiving commands

    properties
        BAUDRATE
        DEVICENAME
        port_num

        lib_name
        notfound
        warnings

        motor1                  %XL430
        motor1_ID       = 1;
        motor1_offset   = 180;
        motor1_axis     = 1;
        
        motor2                  %XL430
        motor2_ID       = 2;
        motor2_offset   = 180;
        motor2_axis     = -1;
        
        motor3                  %XL430
        motor3_ID       = 3;
        motor3_offset   = 180;
        motor3_axis     = -1;
        
        motor4                  %XL320
        motor4_ID       = 4;
        motor4_offset   %= 180;
        motor4_axis     %= 1;
        
        motor5                  %XL320
        motor5_ID       = 5;
        motor5_offset   %= 180;
        motor5_axis     %= 1;
        
        motorE                  %XL320
        motorE_ID       = 6;
        motor6_offset   %= 180;
        motor6_axis     %= 1;
        
    end

    methods
        function obj = motorControl()
            %MOTORCONTROL Construct an instance of this class
            %   Detailed explanation goes here
            obj.startConnection();
            obj.startMotors();
        end

        function startConnection(obj)
            %% Load Libraries
            obj.lib_name = '';

            if strcmp(computer, 'PCWIN')
                obj.lib_name = 'dxl_x86_c';
            elseif strcmp(computer, 'PCWIN64')
                obj.lib_name = 'dxl_x64_c';
            elseif strcmp(computer, 'GLNX86')
                obj.lib_name = 'libdxl_x86_c';
            elseif strcmp(computer, 'GLNXA64')
                obj.lib_name = 'libdxl_x64_c';
            elseif strcmp(computer, 'MACI64')
                obj.lib_name = 'libdxl_mac_c';
            end

            % Load Libraries
            if ~libisloaded(obj.lib_name)
                [obj.notfound, obj.warnings] = loadlibrary(obj.lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
            end


            %% Open Port and set Baud Rate
            obj.BAUDRATE = 57600; % 1000000
            obj.DEVICENAME = 'COM3';

            % Initialize PortHandler Structs
            % Set the port path
            % Get methods and members of PortHandlerLinux or PortHandlerWindows
            obj.port_num = portHandler(obj.DEVICENAME);

            % Initialize PacketHandler Structs
            packetHandler();

            % Open port
            if (openPort(obj.port_num))
                fprintf('Succeeded to open the port!\n');
            else
                unloadlibrary(obj.lib_name);
                fprintf('Failed to open the port!\n');
                input('Press any key to terminate...\n');
                return;
            end
            % Set port baudrate
            if (setBaudRate(obj.port_num, obj.BAUDRATE))
                fprintf('Succeeded to change the baudrate!\n');
            else
                unloadlibrary(obj.lib_name);
                fprintf('Failed to change the baudrate!\n');
                input('Press any key to terminate...\n');
                return;
            end

            % Initialize PacketHandler Structs
            packetHandler();
        end

        function startMotors(obj)
            obj.motor1 = XL430(obj.motor1_ID,obj.port_num, 0, 360);
            obj.motor2 = XL430(obj.motor2_ID,obj.port_num, 0, 360);
            obj.motor3 = XL430(obj.motor3_ID,obj.port_num, 0, 360);
%             obj.motor4 = XL320(obj.motor4_ID,obj.port_num, 0, 300);
%             obj.motor5 = XL320(obj.motor5_ID,obj.port_num, 0, 300);
%             obj.motorE = XL320(obj.motorE_ID,obj.port_num, 0, 300);
        end

        function enableTorque(obj)
            obj.motor1.torqueEnable;
            obj.motor2.torqueEnable;
            obj.motor3.torqueEnable;
%             obj.motor4.torqueEnable;
%             obj.motor5.torqueEnable;
%             obj.motorE.torqueEnable;
        end
        
        function disableTorque(obj)
            obj.motor1.torqueDisable;
            obj.motor2.torqueDisable;
            obj.motor3.torqueDisable;
%             obj.motor4.torqueDisable;
%             obj.motor5.torqueDisable;
%             obj.motorE.torqueDisable;
        end
        
        function positionMode(obj)
            obj.motor1.positionMode; 
            obj.motor2.positionMode;
            obj.motor3.positionMode;
%             obj.motor4.positionMode;
%             obj.motor5.positionMode;
%             obj.motorE.positionMode;
        end
        
        function velocityMode(obj)
            obj.motor1.velocityMode; 
            obj.motor2.velocityMode; 
            obj.motor3.velocityMode; 
%             obj.motor4.velocityMode; 
%             obj.motor5.velocityMode; 
%             obj.motorE.velocityMode; 
        end

        function endConnection(obj)
            obj.motor1.torqueDisable;
            obj.motor2.torqueDisable;
            obj.motor3.torqueDisable;
%             obj.motor4.torqueDisable;
%             obj.motor5.torqueDisable;
%             obj.motorE.torqueDisable;

            % Close port
            calllib(obj.lib_name, 'closePort', obj.port_num);

            % Unload Library
            unloadlibrary(obj.lib_name);
        end

        function setAngles(obj,q1,q2,q3,q4,q5,speed)
            % Inputs: q_n - the desired motor angles, degrees
            %         speed - the speed at which each joint will move, RPM
        
            obj.motor1.setPos(obj.motor1_axis*q1+obj.motor1_offset,speed);
            obj.motor2.setPos(obj.motor2_axis*q2+obj.motor2_offset,speed);
            obj.motor3.setPos(obj.motor3_axis*q3+obj.motor3_offset,speed);
%             obj.motor4.setPos(obj.motor4_axis*q4+obj.motor4_offset,speed);
%             obj.motor5.setPos(obj.motor5_axis*q5+obj.motor5_offset,speed);
        end
        
        function [q1,q2,q3,q4,q5] = getAngles(obj)
            q1 = (obj.motor1.getPos()-obj.motor1_offset)/obj.motor1_axis;
            q2 = (obj.motor2.getPos()-obj.motor2_offset)/obj.motor2_axis;
            q3 = (obj.motor3.getPos()-obj.motor3_offset)/obj.motor3_axis;
%             q4 = (obj.motor4.getPos()-obj.motor4_offset)/obj.motor4_axis;
%             q5 = (obj.motor5.getPos()-obj.motor5_offset)/obj.motor5_axis;
            q4 = -(q2+q3); % TEMPORARY - REPLACE WHEN MOTORS CONNECTED
            q5 = 0; % TEMPORARY - REPLACE WHEN MOTORS CONNECTED
        end
        
        function setVelocities(obj,q1_dot,q2_dot,q3_dot,q4_dot,q5_dot)
            degreesPerS_to_RPM = 1/6; 
            obj.motor1.setVelocity(q1_dot*obj.motor1_axis*degreesPerS_to_RPM);
            obj.motor2.setVelocity(q2_dot*obj.motor2_axis*degreesPerS_to_RPM);
            obj.motor3.setVelocity(q3_dot*obj.motor3_axis*degreesPerS_to_RPM);
%             obj.motor4.setVelocity(q4_dot*obj.motor4_axis*degreesPerS_to_RPM);
%             obj.motor5.setVelocity(q5_dot*obj.motor5_axis*degreesPerS_to_RPM);
        end
        
        function [q1_dot,q2_dot,q3_dot,q4_dot,q5_dot] = getVelocities(obj)
            RPM_to_degreesPerS = 6;
            q1_dot = obj.motor1.getVelocity()*obj.motor1_axis*RPM_to_degreesPerS;
            q2_dot = obj.motor2.getVelocity()*obj.motor2_axis*RPM_to_degreesPerS;
            q3_dot = obj.motor3.getVelocity()*obj.motor3_axis*RPM_to_degreesPerS;
%             q4_dot = obj.motor4.getVelocity()*RPM_to_degreesPerS;
%             q5_dot = obj.motor5.getVelocity()*RPM_to_degreesPerS;
            q4_dot = 0; 
            q5_dot = 0; 
        end
    end

end
