classdef jTower
    %{
        Assignemnt Group 20
        Tower Class, represents the jenga tower to be built, generates and
        stores blocks. 
    
        Required improvements
        - Add functionality to change tower build orientation
        - depending on the potition.
        
    
    %}
    
    %% Properties
    properties (Constant)
        NumBlocks = 54;     % Total blocks in tower
        BPerLayer = 3;      % Blocks Per Layer
        Layers    = 18;     
        Width     = 0.075;  % m
        Height    = 0.270;  % m
        CrossAngle = pi/2   % radians
        
        % if build radius < inversion limit, build from in to out
        InversionX = 0.2; 
        InversionY = -0.15;
    end
    
    properties %(Access = private)
        x_loc       % x coordinate of centre of tower
        y_loc       % y coordinate of centre of tower
        theta       % angle of tower in radians
        complete    % Boolean, to check if the tower is complete
        
        
        towerBlocks % Array to contain all blocks in the tower
    end
    
    
    %% Public Methods  
    methods
        % Constructor
        function obj = jTower(x, y, theta)
            obj.x_loc = x;
            obj.y_loc = y;
            obj.theta = theta;
            
            obj.towerBlocks = obj.constructTowerBlocks;
            obj.complete = false; %tower not yet built
        end
        
        function C = isComplete(obj)
            C = obj.complete;
        end
        
        function B = nextBlock(obj)
            block = 1;
            while obj.towerBlocks(block).isPlaced
                block = block + 1;
            end
            B = obj.towerBlocks(block);
        end
    
    end
    
  
    %% Private Methods
    methods(Access = private)
        
        % Adds all blocks to the tower
        function tower = constructTowerBlocks(obj)
            tower = [];
            for layer = 1:obj.Layers
                for pos = 1:obj.BPerLayer
                    P = obj.blockPosition(layer, pos); 
                    tower = [tower, jBlock(P(1), P(2), P(3), P(4), pos)]; 
                end
            end
        end

        
        % calculates coordiantes of a block from it's place in the tower
        % output matches jBlock constructor
        function P = blockPosition(obj, layer, Pos)
            if mod(layer, 2) % odd layer
                % datum is in centre of brick
                x = obj.blockOffset(Pos, ~obj.isInverse);
                y = 0;
                bTheta = -obj.CrossAngle; %odd layers are cross-lay
                % if tower is inverted, invert cross lay blocks
                bTheta = bTheta + pi*(obj.isInverse);
            else
                % datum is in centre of block
                % Vertical layers allways build from right due to end
                % effector constraints
                y = obj.blockOffset(Pos, 1);
                x = 0;
                bTheta = 0; 
            end

            % z dayum is on lower surface
            z = (layer - 1)*jBlock.Height;

            % Compensate for tower position
            P = obj.zRotation(obj.theta)*[x; y; z];
            P = P + [obj.x_loc; obj.y_loc; 0];
            bTheta = bTheta + obj.theta;
            
            % return column vector
            P = [P; bTheta];

        end
        
        % calculates the block offset from the centre of the centre of the
        % tower from it's position in the layer, and the leading buid edge
        % positiveLead = 1 if leading edge is positive
        function bOffset = blockOffset(obj, layerPos, positiveLead)
            if (positiveLead)
                bOffset = (obj.BPerLayer/2 - layerPos + 0.5)*jBlock.Width;
            else 
                bOffset = -(obj.BPerLayer/2 - layerPos + 0.5)*jBlock.Width;
            end
        end
        
        function I = isInverse(obj)
            if (obj.y_loc > 0)
                I = 1;
            elseif (obj.x_loc < obj.InversionX && ...
                    obj.y_loc > obj.InversionY)
                I = 1;
            else 
                I = 0;
            end 
            
        end
        
        
    end
    
    
    
    methods(Static, Access = private)
        %Just a z rotation, nothing to see here folks
        function R = zRotation(angle)
            R = [cos(angle) -sin(angle) 0;
                 sin(angle) cos(angle) 0;
                 0           0           1];
        end
    
    end
    
end

    
    
    
    
    
    
    
    
    
    