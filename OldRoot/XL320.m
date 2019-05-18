classdef XL320 < handle
    %XL320 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ADDR_PRO_TORQUE_ENABLE      = 24
        ADDR_PRO_GOAL_POSITION      = 30
        ADDR_PRO_GOAL_SPEED         = 32
        ADDR_PRO_CONTROL_MODE       = 11

        ADDR_PRO_PRESENT_POSITION   = 37
        ADDR_PRO_PRESENT_SPEED      = 39
        ADDR_PRO_PRESENT_LOAD       = 41
        ADDR_PRO_PRESENT_VOLTAGE    = 45
        ADDR_PRO_PRESENT_TEMP       = 46
        
        ADDR_PRO_CW_LIMIT           = 6
        ADDR_PRO_CCW_LIMIT          = 8
        
        LEN_PRO_GOAL_SPEED          = 2
        LEN_PRO_PRESENT_POSITION    = 2
        
        
        PROTOCOL_VERSION            = 2.0
        
        DXL_ID
        port_num
        
        TORQUE_ENABLE               = 1;            % Value for enabling the torque
        TORQUE_DISABLE              = 0;            % Value for disabling the torque
        DXL_MINIMUM_POSITION_VALUE  = 0;            % Dynamixel will rotate between this value
        DXL_MAXIMUM_POSITION_VALUE  = 1023;         % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold
        WHEEL_MODE                  = 1;            % Wheel mode (Speed control)
        JOINT_MODE                  = 2;            % Joint mode (Position control)
        
        COMM_SUCCESS                = 0;            % Communication Success result value
        COMM_TX_FAIL                = -1001;        % Communication Tx Failed
        
        dxl_error                   = 0;            % Dynamixel error
        dxl_present_position        = 0;            % Present position
        dxl_comm_result             = 0;            % Communication result
        
        angleOffset                                 % Difference in motor 0 and desired 0
        axis                                        % 1 if z-axis correct, -1 if opposite
        degreeConversionConstant    = 0.29;         % Degrees
        velocityConversionConstant  = 0.111/5;        % RPM but not really because 320s dont have velocity control
    end
    
    methods
        function obj = XL320(ID, port_num, min_angle, max_angle, angleOffset, axis)
            %XL320 Construct an instance of this class
            obj.port_num = port_num;
            obj.DXL_ID = ID;
            obj.dxl_comm_result = obj.COMM_TX_FAIL;
            obj.DXL_MINIMUM_POSITION_VALUE = floor(min_angle/obj.degreeConversionConstant);
            obj.DXL_MAXIMUM_POSITION_VALUE = floor(max_angle/obj.degreeConversionConstant);
            obj.angleOffset = angleOffset;
            obj.axis = axis; 
        end
        
        function [obj] = torqueEnable(obj)
            % Enable Motor Torque
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_TORQUE_ENABLE, obj.TORQUE_ENABLE);
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
        end
        
        function [obj] = torqueDisable(obj)
            % Disable  Motor Torque
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_TORQUE_ENABLE, obj.TORQUE_DISABLE);
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
        end
        
        function [obj] = velocityMode(obj)
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_CONTROL_MODE, obj.WHEEL_MODE);
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
        end
        
        function [obj] = positionMode(obj)
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_CONTROL_MODE, obj.JOINT_MODE);
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
%             % min angle (CW)
%             write2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_CW_LIMIT, obj.DXL_MINIMUM_POSITION_VALUE);
%             [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
%             % max angle (CCW)
%             write2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_CCW_LIMIT, obj.DXL_MAXIMUM_POSITION_VALUE);
%             [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
        end
        
        function [pos, obj] = getPos(obj)
            presentPos = read2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_PRESENT_POSITION);
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
            pos = presentPos*obj.degreeConversionConstant;
        end
        
        function [obj] = setPos(obj, goalPos, speed) % goalPos in degrees
            goalPosCommand = floor(goalPos/obj.degreeConversionConstant); % Convert to motor command
            write2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_GOAL_POSITION, typecast(int16(goalPosCommand), 'uint16'));
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
        end
        
        function [dxl_present_speed, obj] = getSpeed(obj)
            dxl_present_speed = read2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_PRESENT_SPEED);
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
        end
        
        function [obj] = setVelocity(obj, goalVelocity)
            goalVelocityCommand = (goalVelocity<0)*1023+abs(floor(goalVelocity/obj.velocityConversionConstant))
            write2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_GOAL_SPEED, typecast(int16(goalVelocityCommand), 'uint16'));
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
        end
        
    end
    

end

