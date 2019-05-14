classdef motorControl < handle
    %MOTORCONTROL Handles motor control
    %   Initialisation of motors, and sending and receiving commands

    properties
        BAUDRATE
        DEVICENAME
        port_num
        PROTOCOL_VERSION = 2.0

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
        motor4_offset   = 150;%= 180;
        motor4_axis     = 1;%= 1;
        
        motor5                  %XL320
        motor5_ID       = 5;
        motor5_offset   = 150; %= 180;
        motor5_axis     = 1; %= 1;
        
        motor6                  %XL320
        motor6_ID       = 6;
        motor6_offset   = 150; %= 180;
        motor6_axis     = 1; %= 1;
        
        COMM_SUCCESS                = 0;            % Communication Success result value
        COMM_TX_FAIL                = -1001;        % Communication Tx Failed
        
        dxl_error                   = 0;            % Dynamixel error
        dxl_comm_result             = 0;            % Communication result
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
            obj.motor1 = XL430(obj.motor1_ID,obj.port_num, 0, 360, obj.motor1_offset, obj.motor1_axis);
            obj.motor2 = XL430(obj.motor2_ID,obj.port_num, 0, 360, obj.motor2_offset, obj.motor2_axis);
            obj.motor3 = XL430(obj.motor3_ID,obj.port_num, 0, 360, obj.motor3_offset, obj.motor3_axis);
            obj.motor4 = XL320(obj.motor4_ID,obj.port_num, 0, 300, obj.motor4_offset, obj.motor4_axis);
            obj.motor5 = XL320(obj.motor5_ID,obj.port_num, 0, 300, obj.motor5_offset, obj.motor5_axis);
%             obj.motor6 = XL320(obj.motor6_ID,obj.port_num, 0, 300, obj.motor6_offset, obj.motor6_axis);
        end

        function enableTorque(obj)
            obj.motor1.torqueEnable;
            obj.motor2.torqueEnable;
            obj.motor3.torqueEnable;
            obj.motor4.torqueEnable;
            obj.motor5.torqueEnable;
%             obj.motor6.torqueEnable;
        end
        
        function disableTorque(obj)
            obj.motor1.torqueDisable;
            obj.motor2.torqueDisable;
            obj.motor3.torqueDisable;
            obj.motor4.torqueDisable;
            obj.motor5.torqueDisable;
%             obj.motor6.torqueDisable;
        end
        
        function positionMode(obj)
            obj.motor1.positionMode; 
            obj.motor2.positionMode;
            obj.motor3.positionMode;
            obj.motor4.positionMode;
            obj.motor5.positionMode;
%             obj.motor6.positionMode;
        end
        
        function velocityMode(obj)
            obj.motor1.velocityMode; 
            obj.motor2.velocityMode; 
            obj.motor3.velocityMode; 
            obj.motor4.velocityMode; 
            obj.motor5.velocityMode; 
%             obj.motor6.velocityMode; 
        end

        function endConnection(obj)
            obj.motor1.torqueDisable;
            obj.motor2.torqueDisable;
            obj.motor3.torqueDisable;
            obj.motor4.torqueDisable;
            obj.motor5.torqueDisable;
%             obj.motor6.torqueDisable;

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
            obj.motor4.setPos(obj.motor4_axis*q4+obj.motor4_offset,speed);
            obj.motor5.setPos(obj.motor5_axis*q5+obj.motor5_offset,speed);
        end
        
        function [q1,q2,q3,q4,q5] = getAngles(obj)
            q1 = (obj.motor1.getPos()-obj.motor1_offset)/obj.motor1_axis;
            q2 = (obj.motor2.getPos()-obj.motor2_offset)/obj.motor2_axis;
            q3 = (obj.motor3.getPos()-obj.motor3_offset)/obj.motor3_axis;
            q4 = (obj.motor4.getPos()-obj.motor4_offset)/obj.motor4_axis;
            q5 = (obj.motor5.getPos()-obj.motor5_offset)/obj.motor5_axis;
%             q4 = -(q2+q3); % TEMPORARY - REPLACE WHEN MOTORS CONNECTED
%             q5 = 0; % TEMPORARY - REPLACE WHEN MOTORS CONNECTED
        end
        
        function setVelocities(obj,q1_dot,q2_dot,q3_dot,q4_dot,q5_dot)
            degreesPerS_to_RPM = 1/6; 
            obj.motor1.setVelocity(q1_dot*obj.motor1_axis*degreesPerS_to_RPM);
            obj.motor2.setVelocity(q2_dot*obj.motor2_axis*degreesPerS_to_RPM);
            obj.motor3.setVelocity(q3_dot*obj.motor3_axis*degreesPerS_to_RPM);
            obj.motor4.setVelocity(q4_dot*obj.motor4_axis*degreesPerS_to_RPM);
            obj.motor5.setVelocity(q5_dot*obj.motor5_axis*degreesPerS_to_RPM);
        end
        
        function [q1_dot,q2_dot,q3_dot,q4_dot,q5_dot] = getVelocities(obj)
            RPM_to_degreesPerS = 6;
            q1_dot = obj.motor1.getVelocity()*obj.motor1_axis*RPM_to_degreesPerS;
            q2_dot = obj.motor2.getVelocity()*obj.motor2_axis*RPM_to_degreesPerS;
            q3_dot = obj.motor3.getVelocity()*obj.motor3_axis*RPM_to_degreesPerS;
            q4_dot = obj.motor4.getVelocity()*obj.motor4_axis*RPM_to_degreesPerS;
            q5_dot = obj.motor5.getVelocity()*obj.motor5_axis*RPM_to_degreesPerS;
%             q4_dot = 0; 
%             q5_dot = 0; 
        end
        
        function [t_act,q_act] = executeTrajectory(obj,t,q,q_dot, K)
            % This Method executes control of a specified trajectory based
            % on joint angles and velocities
            % Inputs:
            %           t - nx1 vector of time values
            %           q - nx5 matrix of joint angles (deg)
            %           q_dot - nx5 matrix of joint velocities (deg/s)
            %           K - 1x5 vector of Controller gains for each motor
            dt = t(2) - t(1); 
            
            t_act = zeros(size(t));
            q_act = zeros(size(q)); 

%             qMot = zeros(size(q));
%             q_dot_Mot = zeros(size(q)); 
            
            %% Pre-Converting input values to motor commands
            % Positions
%             qMot(:,1) = (q(:,1)*obj.motor1.axis+obj.motor1_offset)*obj.motor1.degreeConversionConstant;
%             qMot(:,2) = (q(:,2)*obj.motor1.axis+obj.motor2_offset)*obj.motor2.degreeConversionConstant;
%             qMot(:,3) = (q(:,3)*obj.motor1.axis+obj.motor3_offset)*obj.motor3.degreeConversionConstant;
%             qMot(:,4) = (q(:,4)*obj.motor1.axis+obj.motor4_offset)*obj.motor4.degreeConversionConstant;
%             qMot(:,5) = (q(:,5)*obj.motor1.axis+obj.motor5_offset)*obj.motor5.degreeConversionConstant;
            
            % Velocities
%             degreesPerS_to_RPM = 1/6;           
%             q_dot_Mot(:,1) = q_dot(:,1)*degreesPerS_to_RPM*obj.motor1.axis/obj.motor1.velocityConversionConstant;
%             q_dot_Mot(:,2) = q_dot(:,2)*degreesPerS_to_RPM*obj.motor2.axis/obj.motor2.velocityConversionConstant;
%             q_dot_Mot(:,3) = q_dot(:,3)*degreesPerS_to_RPM*obj.motor3.axis/obj.motor3.velocityConversionConstant;
%             q_dot_Mot(:,4) = q_dot(:,4)*degreesPerS_to_RPM*obj.motor4.axis/obj.motor4.velocityConversionConstant;
%             q_dot_Mot(:,5) = q_dot(:,5)*degreesPerS_to_RPM*obj.motor5.axis/obj.motor5.velocityConversionConstant;

            %% GROUP SYNC READ WRITE
            % Initialize Groupsyncwrite Structs
%             groupwrite_num_430 = groupSyncWrite(obj.port_num, obj.PROTOCOL_VERSION, obj.motor1.ADDR_PRO_GOAL_VELOCITY, obj.motor1.LEN_PRO_GOAL_VELOCITY);
%             groupwrite_num_320 = groupSyncWrite(obj.port_num, obj.PROTOCOL_VERSION, obj.motor4.ADDR_PRO_GOAL_SPEED, LEN_PRO_GOAL_SPEED);
% 
% 
            % Initialize Groupsyncread Structs for Present Position
%             groupread_num_430 = groupSyncRead(obj.port_num, obj.PROTOCOL_VERSION, obj.motor1.ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
%             groupread_num_320 = groupSyncRead(obj.port_num, obj.PROTOCOL_VERSION, obj.motor4.ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
                
            % Enable Motor Torques
            obj.disableTorque; 
            obj.velocityMode;
            obj.setVelocities(0,0,0,0,0);
            obj.enableTorque; 
            pause(5)
            
            % Add Parameter Storage for 430s
%             addParameterStoragePositionSYNC(groupread_num_430, obj.motor1.DXL_ID);
%             addParameterStoragePositionSYNC(groupread_num_430, obj.motor2.DXL_ID);
%             addParameterStoragePositionSYNC(groupread_num_430, obj.motor3.DXL_ID);
            
            % 
            dxl_present_time = read2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.motor1.DXL_ID, obj.motor1.ADDR_PRO_REALTIME_TICK);
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
            dxl_start_time = dxl_present_time; %ms 
            
            time_elapsed = 0; 
            
%             % Syncread present position
%             groupSyncReadTxRxPacket(groupread_num_430);
%             if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
%                 printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
%             end
%             
%             % Check if groupsyncread data of Dynamixel#1 is available
%             dxl_getdata_result = groupSyncReadIsAvailable(groupread_num_430, obj.motor1.DXL_ID, obj.motor1.ADDR_PRO_PRESENT_POSITION, obj.motor1.LEN_PRO_PRESENT_POSITION);
%             if dxl_getdata_result ~= true
%                 fprintf('[ID:%03d] groupSyncRead getdata failed', obj.motor1.DXL_ID);
%                 return;
%             end
%             % Check if groupsyncread data of Dynamixel#2 is available
%             dxl_getdata_result = groupSyncReadIsAvailable(groupread_num_430, obj.motor2.DXL_ID, obj.motor2.ADDR_PRO_PRESENT_POSITION, obj.motor2.LEN_PRO_PRESENT_POSITION);
%             if dxl_getdata_result ~= true
%                 fprintf('[ID:%03d] groupSyncRead getdata failed', obj.motor2.DXL_ID);
%                 return;
%             end
%             % Check if groupsyncread data of Dynamixel#3 is available
%             dxl_getdata_result = groupSyncReadIsAvailable(groupread_num_430, obj.motor3.DXL_ID, obj.motor3.ADDR_PRO_PRESENT_POSITION, obj.motor3.LEN_PRO_PRESENT_POSITION);
%             if dxl_getdata_result ~= true
%                 fprintf('[ID:%03d] groupSyncRead getdata failed', obj.motor3.DXL_ID);
%                 return;
%             end
%             % Check if groupsyncread data of Dynamixel#4 is available
%             dxl_getdata_result = groupSyncReadIsAvailable(groupread_num_320, obj.motor4.DXL_ID, obj.motor4.ADDR_PRO_PRESENT_POSITION, obj.motor4.LEN_PRO_PRESENT_POSITION);
%             if dxl_getdata_result ~= true
%                 fprintf('[ID:%03d] groupSyncRead getdata failed', obj.motor4.DXL_ID);
%                 return;
%             end
%             % Check if groupsyncread data of Dynamixel#5 is available
%             dxl_getdata_result = groupSyncReadIsAvailable(groupread_num_320, obj.motor5.DXL_ID, obj.motor5.ADDR_PRO_PRESENT_POSITION, obj.motor5.LEN_PRO_PRESENT_POSITION);
%             if dxl_getdata_result ~= true
%                 fprintf('[ID:%03d] groupSyncRead getdata failed', obj.motor5.DXL_ID);
%                 return;
%             end
            
            % Get Dynamixel#1 present position value
%             dxl1_present_position = groupSyncReadGetData(groupread_num_430, obj.motor1.DXL_ID, obj.motor1.ADDR_PRO_PRESENT_POSITION, obj.motor1.LEN_PRO_PRESENT_POSITION);
%             dxl2_present_position = groupSyncReadGetData(groupread_num_430, obj.motor2.DXL_ID, obj.motor2.ADDR_PRO_PRESENT_POSITION, obj.motor2.LEN_PRO_PRESENT_POSITION);
%             dxl3_present_position = groupSyncReadGetData(groupread_num_430, obj.motor3.DXL_ID, obj.motor3.ADDR_PRO_PRESENT_POSITION, obj.motor3.LEN_PRO_PRESENT_POSITION);
%             dxl4_present_position = groupSyncReadGetData(groupread_num_320, obj.motor4.DXL_ID, obj.motor4.ADDR_PRO_PRESENT_POSITION, obj.motor4.LEN_PRO_PRESENT_POSITION);
%             dxl5_present_position = groupSyncReadGetData(groupread_num_320, obj.motor5.DXL_ID, obj.motor5.ADDR_PRO_PRESENT_POSITION, obj.motor5.LEN_PRO_PRESENT_POSITION);
%             q_error_1 = q(1,1) - dxl1_present_position;
%             q_error_2 = q(1,2) - dxl2_present_position;
%             q_error_3 = q(1,3) - dxl3_present_position;

            [q1,q2,q3,q4,q5] = obj.getAngles;
            q_error_1 = q(1,1) - q1;
            q_error_2 = q(1,2) - q2;
            q_error_3 = q(1,3) - q3;
            q_error_4 = q(1,4) - q4;
            q_error_5 = q(1,5) - q5;
            go = 1; 
            logCounter = 1; 
            
                 
            % Run Commands on robot
            while time_elapsed<t(end) && go
                
                 
                % Get Current Time from motor
                dxl_present_time = read2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.motor1.DXL_ID, obj.motor1.ADDR_PRO_REALTIME_TICK);
                time_elapsed = dxl_present_time-dxl_start_time;
                if time_elapsed<0 
                    time_elapsed = time_elapsed+obj.motor1.DXL_MAXIMUM_TICK;
                end
                time_elapsed = time_elapsed/1000;
                
                % Find command Velocities from trajectory matrix
                i = 1;
                while go && i<=length(t) && t(i)<time_elapsed
                    i = i+1; 
                    if i>=length(t)
                        go = 0;
                        i = length(t);
                    end
                end
                
                [q1,q2,q3,q4,q5] = obj.getAngles;
                
                q_error_1 = q(i,1) - q1;
                q_error_2 = q(i,2) - q2;
                q_error_3 = q(i,3) - q3;
                q_error_4 = q(1,4) - q4;
                q_error_5 = q(1,5) - q5;
                
                q_dot1 = q_dot(i,1)+K(1)*q_error_1;
                q_dot2 = q_dot(i,2)+K(2)*q_error_2;
                q_dot3 = q_dot(i,3)+K(3)*q_error_3;
                q_dot4 = q_dot(i,4)+K(4)*q_error_4;
                q_dot5 = q_dot(i,5)+K(5)*q_error_5;
                
                obj.setVelocities(q_dot1,q_dot2,q_dot3,q_dot4,q_dot5);
                
                t_act(logCounter) = time_elapsed;
                q_act(logCounter,1) = q1;
                q_act(logCounter,2) = q2;
                q_act(logCounter,3) = q3;
                q_act(logCounter,4) = q4;
                q_act(logCounter,5) = q5;
                logCounter = logCounter+1;
                
                
%                 q_dot4 = q_dot_Mot(i,4);
%                 q_dot5 = q_dot_Mot(i,5);
                

%                 % Add goal velocity value to the Syncwrite Storage
%                  addGoalVelocitySYNC(groupwrite_num_430, obj.motor1.DXL_ID, q_dot1, obj.motor1.LEN_PRO_GOAL_VELOCITY)
%                  addGoalVelocitySYNC(groupwrite_num_430, obj.motor2.DXL_ID, q_dot2, obj.motor2.LEN_PRO_GOAL_VELOCITY)
%                  addGoalVelocitySYNC(groupwrite_num_430, obj.motor3.DXL_ID, q_dot3, obj.motor3.LEN_PRO_GOAL_VELOCITY)
%                  
%                  % Syncwrite Goal Velocity
%                  groupSyncWriteTxPacket(groupwrite_num_430);
%                  if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= COMM_SUCCESS
%                      printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
%                  end
%                  
%                  groupSyncWriteClearParam(groupwrite_num_430);
%                  
%                  groupSyncReadTxRxPacket(groupread_num_430);
%                  dxl1_present_position = groupSyncReadGetData(groupread_num_430, obj.motor1.DXL_ID, obj.motor1.ADDR_PRO_PRESENT_POSITION, obj.motor1.LEN_PRO_PRESENT_POSITION);
%                  dxl2_present_position = groupSyncReadGetData(groupread_num_430, obj.motor2.DXL_ID, obj.motor2.ADDR_PRO_PRESENT_POSITION, obj.motor2.LEN_PRO_PRESENT_POSITION);
%                  dxl3_present_position = groupSyncReadGetData(groupread_num_430, obj.motor3.DXL_ID, obj.motor3.ADDR_PRO_PRESENT_POSITION, obj.motor3.LEN_PRO_PRESENT_POSITION);
                 
            end
            
           obj.setVelocities(0,0,0,0,0);

            %% GROUP BULK READ WRITE
            
            % Initialize groupBulkWrite Struct
%             groupwrite_num = groupBulkWrite(obj.port_num, obj.PROTOCOL_VERSION);
            
            % Initialize Groupbulkread Structs
%             groupread_num = groupBulkRead(obj.port_num, obj.PROTOCOL_VERSION);
            
            % Add Parameters for Storage (Position) 
%             addParameterStoragePosition(obj);
        end
        
        function addParameterStoragePositionBULK(obj)
            % Add parameter storage for motor 1 position value
            dxl_addparam_result = groupBulkReadAddParam(groupread_num, obj.motor1.DXL_ID, obj.motor1.ADDR_PRO_PRESENT_POSITION, obj.motor1.LEN_PRO_PRESENT_POSITION);
            if dxl_addparam_result ~= true
                fprintf('[ID:%03d] groupBulkRead addparam failed', obj.motor1.DXL_ID);
                return;
            end
            
            % Add parameter storage for motor 2 position value
            dxl_addparam_result = groupBulkReadAddParam(groupread_num, obj.motor1.DXL_ID, obj.motor1.ADDR_PRO_PRESENT_POSITION, obj.motor1.LEN_PRO_PRESENT_POSITION);
            if dxl_addparam_result ~= true
                fprintf('[ID:%03d] groupBulkRead addparam failed', obj.motor1.DXL_ID);
                return;
            end
            
            % Add parameter storage for motor 3 position value
            dxl_addparam_result = groupBulkReadAddParam(groupread_num, obj.motor1.DXL_ID, obj.motor1.ADDR_PRO_PRESENT_POSITION, obj.motor1.LEN_PRO_PRESENT_POSITION);
            if dxl_addparam_result ~= true
                fprintf('[ID:%03d] groupBulkRead addparam failed', obj.motor1.DXL_ID);
                return;
            end
            
            % Add parameter storage for motor 4 position value
            dxl_addparam_result = groupBulkReadAddParam(groupread_num, obj.motor1.DXL_ID, obj.motor1.ADDR_PRO_PRESENT_POSITION, obj.motor1.LEN_PRO_PRESENT_POSITION);
            if dxl_addparam_result ~= true
                fprintf('[ID:%03d] groupBulkRead addparam failed', obj.motor1.DXL_ID);
                return;
            end
            
            % Add parameter storage for motor 5 position value
            dxl_addparam_result = groupBulkReadAddParam(groupread_num, obj.motor1.DXL_ID, obj.motor1.ADDR_PRO_PRESENT_POSITION, obj.motor1.LEN_PRO_PRESENT_POSITION);
            if dxl_addparam_result ~= true
                fprintf('[ID:%03d] groupBulkRead addparam failed', obj.motor1.DXL_ID);
                return;
            end
            
            % Add parameter storage for motor 6 position value
            dxl_addparam_result = groupBulkReadAddParam(groupread_num, obj.motor1.DXL_ID, obj.motor1.ADDR_PRO_PRESENT_POSITION, obj.motor1.LEN_PRO_PRESENT_POSITION);
            if dxl_addparam_result ~= true
                fprintf('[ID:%03d] groupBulkRead addparam failed', obj.motor1.DXL_ID);
                return;
            end
        end
        
                 
    end
    
    methods(Static)
        function addParameterStoragePositionSYNC(groupread_num, DXL_ID)
            dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL_ID);
            if dxl_addparam_result ~= true
              fprintf('[ID:%03d] groupSyncRead addparam failed', DXL_ID);
              return;
            end
        end
        
        function addGoalVelocitySYNC(groupwrite_num, DXL_ID, vel, LEN_PRO_GOAL_VELOCITY)
            dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL_ID, typecast(int32(vel), 'uint32'), LEN_PRO_GOAL_VELOCITY);
            if dxl_addparam_result ~= true
                fprintf('[ID:%03d] groupSyncWrite addparam failed', DXL1_ID);
                return;
            end
        end
    end
end
