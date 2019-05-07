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
        Length = 75;    % mm
        Width = 25;     % mm
        Height = 15;    % mm
    end
    
    properties(Access = private)
        placed      % Boolean value to represent if brick has been placed
        x           % x coordinate of brick
        y           % y coordinate of brick
        z           % z coordinate of brick
        theta       % orientation of brick
    end
    
    
    %% Constructor
    methods 
       
        function obj = jBlock(x, y, z, theta)
            obj.placed = false;
            obj.x = x;
            obj.y = y;
            obj.z = z;
            obj.theta = theta;
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
        
        %% Mutators
        function placeBlock(obj)
            obj.placed = true;
        end
        
    end
    
end
