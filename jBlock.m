classdef jBlock < handle
    %{
        Assignment Group 20
        Block class: represents an individual jenga block as part of the
        jTower class
        
        Coordinate datum of brick is recognised as the middle of the brick 
        on the lower surface. 
    %}
    properties (Constant)
        Material = 'alder';
        Length = 0.075;    % m
        Width = 0.025;     % m
        Height = 0.015;    % m
        
        approachX = 4; % used in calculating approach (block widths)
        approachZ = 2; % used in calculating approach (block hieghts)
        dropHeight = 0.005; % height to drop the block from

        dropIncrement = 0.0005 % scale drop height with z position
    end
    
    properties(Access = private)
        placed      % Boolean value to represent if brick has been placed
        x           % x coordinate of brick
        y           % y coordinate of brick
        z           % z coordinate of brick
        theta       % orientation of brick
        
        layerPos        % order in which the block is placed in the layer
        approachPos     % via to approach tower
        dropLoc    % dropLocation
    end
    
    
    %% Constructor
    methods 
       
        function obj = jBlock(x, y, z, theta, layerPos)
            obj.placed = false;
            obj.x = x;
            obj.y = y;
            obj.z = z;
            obj.theta = theta;
            obj.layerPos = layerPos;
            obj.approachPos = obj.calculateApproach;
            obj.dropLoc = obj.calculateDrop;
        end
        
        %% Accessors
        % Returns coordinates of block in 0 frame
        function P = getPosition(obj)
            P = [obj.x; obj.y; obj.z; obj.theta];
        end
        
        % Check if brick has been placed
        function P = isPlaced(obj)
            P = obj.placed;
        end
        
        % Return via position for approaching the brick
        function A = approachPosition(obj)
            A = obj.approachPos;
        end
        
        function D = dropLocation(obj)
            D = obj.dropLoc;
        end
            
        
        %% Mutators
        function placeBlock(obj)
            obj.placed = true;
        end
        
    end
    
    methods (Access = private)
        
        % Calculate approach via position
        % Uses blocks position in the tower to calculate a via that will
        % give clearance when placing the block and returning. 
        % returns coordinate [x, y, z, theta]
        function A = calculateApproach(obj)
            Pos = obj.getPosition;
            XClearance = (obj.approachX-obj.layerPos)*(obj.Width);
            ZClearance = obj.approachZ*obj.Height;
            r_pa = [0; -XClearance; ZClearance];
            A = Pos + [obj.zRotation(obj.theta)*r_pa; 0];
        end
    
        function D = calculateDrop(obj)
            D = obj.getPosition + [0; 0; obj.dropHeight; 0];
            % scale for z position
            OffsetZ = (obj.z/obj.Height)*obj.dropIncrement;
            D = D + [0; 0; OffsetZ; 0];
        end
        
    end
    
    
    
    methods (Static, Access=private)
        % Just a z rotation, nothing to see here folks
        % takes input in radians
        function R = zRotation(theta)
            R = [cos(theta) -sin(theta) 0;
                 sin(theta) cos(theta) 0;
                 0           0         1];
        end
    end
end






