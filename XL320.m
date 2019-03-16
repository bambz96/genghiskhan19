classdef XL320
    %XL320 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ADDR_PRO_TORQUE_ENABLE      = 24
        ADDR_PRO_GOAL_POSITION      = 30
        ADDR_PRO_PRESENT_POSITION   = 37
        
        PROTOCOL_VERSION            = 2.0
        
        DXL_ID
        port_num
        
        TORQUE_ENABLE               = 1;            % Value for enabling the torque
        TORQUE_DISABLE              = 0;            % Value for disabling the torque
        DXL_MINIMUM_POSITION_VALUE  = 100;          % Dynamixel will rotate between this value
        DXL_MAXIMUM_POSITION_VALUE  = 1000;         % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold
        
        COMM_SUCCESS                = 0;            % Communication Success result value
        COMM_TX_FAIL                = -1001;        % Communication Tx Failed
        
        dxl_error                   = 0;            % Dynamixel error
        dxl_present_position        = 0;            % Present position
        dxl_comm_result             = 0;            % Communication result
    end
    
    methods
        function obj = XL320(ID, port_num)
            %XL320 Construct an instance of this class
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
        
        function [dxl_present_position, obj] = getPos(obj)
            dxl_present_position = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_PRESENT_POSITION);
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
        end
        
        function [obj] = setPos(obj, goalPos)
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID, obj.ADDR_PRO_GOAL_POSITION, typecast(int32(goalPos), 'uint32'));
            [obj.dxl_comm_result, obj.dxl_error] = checkComms(obj.port_num, obj.PROTOCOL_VERSION);
        end
        
    end
end

