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
        Width     = 75;
        Height    = 270;  
    end
    
    properties (Access = private)
        x_loc       % x coordinate of centre of tower
        y_loc       % y coordinate of centre of tower
        theta       % z coordinate of centre of tower
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
            while obj.towerBlocks(block).isPlaced;
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
                    P = obj.blockPosition(layer,pos); 
                    tower = [tower, jBlock(P(1), P(2), P(3), P(4))]; 
                end
            end
        end

        % calculates coordiantes of a block from it's place in the tower
        % output matches jBlock constructor
        function P = blockPosition(obj, layer, layerPos)
            if mod(layer, 2) % odd layer
                % datum is in centre of brick
                x = (obj.BPerLayer/2 - layerPos + 0.5)*jBlock.Width;
                y = 0;
                theta = 90; %even layers are cross-lay
            else
                %datum i sin centre of block
                y = (layerPos - obj.BPerLayer/2 -0.5)*jBlock.Width;
                x = 0;
                theta = 0; 
            end

            % z dayum is on lower surface
            z = (layer - 1)*jBlock.Height;

            % Compensate for tower position
            P = obj.zRotation(obj.theta)*[x; y; z];
            P = P + [obj.x_loc; obj.y_loc; 0];
            theta = theta + obj.theta;
            
            % return column vector
            P = [P; theta];

        end
    end
    
    methods(Static, Access = private)
        %Just a z rotation, nothing to see here folks
        function R = zRotation(theta)
            R = [cosd(theta) sind(theta) 0;
                -sind(theta) cosd(theta) 0;
                 0           0           1];
        end
    
    end
    
end

    
    
    
    
    
    
    
    
    
    