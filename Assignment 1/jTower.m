classdef jTower
    %{
        Assignemnt Group 20
        Tower Class, represents the jenga tower to be built, generates and 
        
        blocks. 
    
    %}
    
    properties (Constant)
        NumBlocks = 54;
    end
    
    properties (Access = private)
        x_loc       % x coordinate of centre of tower
        y_loc       % y coordinate of centre of tower
        theta       % z coordinate of centre of tower
        complete    % Boolean, to check if the tower is complete
        
        blocks      % Array to contain all blocks in the tower
    end
    
    function obj = jTower(x, y, theta)
        obj.x_loc = x;
        obj.x.loc = y;
        obj.theta = theta;
    end
    
end
