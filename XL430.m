classdef XL430 < handle
    %XL430 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ADDR_PRO_TORQUE_ENABLE      = 64
        ADDR_PRO_GOAL_POSITION      = 116
        ADDR_PRO_GOAL_VELOCITY      = 104
        ADDR_PRO_OPERATING_MODE     = 11

        ADDR_PRO_PRESENT_POSITION   = 132
        ADDR_PRO_PRESENT_VELOCITY   = 128
        ADDR_PRO_PRESENT_LOAD       = 126
        ADDR_PRO_PRESENT_VOLTAGE    = 144
        ADDR_PRO_PRESENT_TEMP       = 146
        
        
        PROTOCOL_VERSION            = 2.0
        
        DXL_ID
        port_num
        
        TORQUE_ENABLE               = 1;            % Value for enabling the torque
        TORQUE_DISABLE              = 0;            % Value for disabling the torque
        DXL_MINIMUM_POSITION_VALUE  = 0;            % Dynamixel will rotate between this value
        DXL_MAXIMUM_POSITION_VALUE  = 4095;         % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        
        DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold
        VELOCITY_CONTROL            = 1;            % Wheel mode (Speed control)
        POSITION_CONTROL            = 3;            % Joint mode (Position control)
        
        COMM_SUCCESS                = 0;            % Communication Success result value
        COMM_TX_FAIL                = -1001;        % Communication Tx Failed
        
        dxl_error                   = 0;            % Dynamixel error
        dxl_present_position        = 0;            % Present position
        dxl_comm_result             = 0;            % Communication result
        
        degreeConversionConstant    = 0.088;        % Degrees
        velocityConversionConstant  = 0.229         % RPM 
    end
    
    methods
        function obj = XL430(ID, port_num)
            %XL430 Construct an instance of this class
            obj.port_num = port_num;
            obj.DXL_ID = ID;
            obj.dxl_comm_result = obj.COMM_TX_FAIL;
            
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
            obj.torqueDisable;
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_OPERATING_MODE, obj.VELOCITY_CONTROL);
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
            obj.torqueEnable;
        end
        
        function [obj] = positionMode(obj)
            obj.torqueDisable;
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_OPERATING_MODE, obj.POSITION_CONTROL);
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
            obj.torqueEnable;
        end
        
        function [pos, obj] = getPos(obj)
            presentPos = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_PRESENT_POSITION);
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
            pos = presentPos*obj.degreeConversionConstant;
        end
        
        function [obj] = setPos(obj, goalPos) % goalPos in degrees
            obj.positionMode; 
            goalPosCommand = floor(goalPos/obj.degreeConversionConstant); % Convert to motor command
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_GOAL_POSITION, typecast(int32(goalPosCommand), 'uint32'));
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
        end
        
        function [velocity, obj] = getVelocity(obj)
            presentSpeed = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_PRESENT_VELOCITY);
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
            velocity = presentSpeed*obj.velocityConversionConstant; 
        end
        
        function [obj] = setVelocity(obj, goalVelocity) % In RPM
            obj.torqueEnable;
            goalVelocityCommand = floor(goalVelocity/obj.velocityConversionConstant);
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_GOAL_VELOCITY, typecast(int32(goalVelocityCommand), 'uint32'));
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
        end
        
    end
end

