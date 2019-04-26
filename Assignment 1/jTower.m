classdef jTower
    %{
        Assignemnt Group 20
        Tower Class, represents the jenga tower to be built, generates and
        stores blocks. 
    
    %}
    
    %% Properties
    properties (Constant)
        NumBlocks = 54;     % Total blocks in tower
        BPerLayer = 3;      % Blocks Per Layer
        Layers    = 18;     
    end
    
    properties (Access = private)
        x_loc       % x coordinate of centre of tower
        y_loc       % y coordinate of centre of tower
        theta       % z coordinate of centre of tower
        complete    % Boolean, to check if the tower is complete
        
        towerBlocks % Array to contain all blocks in the tower
    end
    
    
    %% Methods  
    methods
        function obj = jTower(x, y, theta)
            obj.x_loc = x;
            obj.y_loc = y;
            obj.theta = theta;
            
            towerBlocks = obj.constructTowerBlocks;
            complete = false; %tower not yet built
        end
    
    end
    
    methods(Access = private)
        
        % Adds all blocks to the tower
        function tower = constructTowerBlocks(obj)
            tower = [];
            for layer = 1:obj.Layers
                for pos = 1:obj.BPerLayer
                    [x, y, z, theta] = obj.blockPosition(layer,pos);
                    tower = [tower, jBlock(x, y, z, theta)];
                end
            end
        end

        % calculates coordiantes of a block from it's place in the tower
        % output matches jBlock constructor
        function [x, y, z, theta] = blockPosition(obj, layer, layerPos)
            if mod(layer, 2) % odd layer
                % datum is in centre of brick
                x = (obj.BPerLayer/2 - layerPos + 0.5)*jBlock.Width;
                y = 0;
                theta = 0;
            else
                %datum i sin centre of block
                y = (layerPos - obj.BPerLayer/2 -0.5)*jBlock.Width;
                x = 0;
                theta = 90; %even layers are cross-lay
            end

            % z dayum is on lower surface
            z = (layer - 1)*jBlock.Height;

            %Compensate for tower position
            x = obj.zRotation(obj.theta)*x + obj.x_loc;
            y = obj.zRotation(obj.theta)*y + obj.y_loc;
            theta = theta + obj.theta;

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

    
    
    
    
    
    
    
    
    
    