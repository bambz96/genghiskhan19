    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    A simple script that generates trajectories for testing
    
    Going to have a crack at combining some trajectories now...

    %}

DOF = 5;

SampleTime = 0.1;

pieces = 1;

DATA = zeros(pieces, 6, DOF);


% Define the loading bay coordinates

yOffset = 0; 
LoadingBay = [37.5; -187.5+yOffset; -1.4; -90; 0];

% Create the tower
Tower = jTower(0.200, 0, 0);

% Generate a block to be delivered
FirstBlock = Tower.nextBlock;

%% Find Drop location for Ronda
dropLocation = [FirstBlock.getPosition; 0] + ...
        [0; 0; moveBlock_trj.DropHeight; 0; 0];

    

%% Calculate a set of trajectories
Greg = grip_trj(LoadingBay, 0, SampleTime);
Mark = moveBlock_trj(LoadingBay, 1, SampleTime, 2, FirstBlock);
Ronda = release_trj(dropLocation, 3, SampleTime); 
Ricky = return_trj(LoadingBay, 4, SampleTime, 2, FirstBlock);



% Combine TimeVals
TimeVals = [Greg.getTime, Mark.getTime, Ronda.getTime, Ricky.getTime];


%% Combine Trajectories
%initialise data to correct size
pieces = Greg.getPieces + Mark.getPieces + Ronda.getPieces + Ricky.getPieces;
DATA = zeros(pieces, 6, DOF);



% Mark = moveBlock_trj(LoadingBay, 0, SampleTime, 2, FirstBlock);
% Mark = grip_trj(LoadingBay, 0, SampleTime); 
Mark = release_trj(FirstBlock, 0, SampleTime); 
% Mark = return_trj(LoadingBay, 0, SampleTime, 2, FirstBlock);



%% Process data into form for Robot

% NOTES: for data structure
% rows: pieces
% columns [a3, a2, a1, a0, ts, tf]
% pages: coordinates

for x = 1:3
    % Get Coefficients for one coordinate trajectory
    
    for p = 1:(Mark.getPieces)
        Coeffs = Mark.getCoefficients;
        for a = 1:4
            DATA(p, a, x) = myShrink(Coeffs(p, a, x)/1000);
        end
    end
    
    for p = 1:(Ricky.getPieces)
        Coeffs = Ricky.getCoefficients;
        for a = 1:4
            DATA(p, a, x) = myShrink(Coeffs(p, a, x)/1000);
        end
    end
        
  % this is a bit of a mess... hmmm...
        
        
        
        DATA(p, 5:6, x) = TimeVals(p:p + 1);
    end
end

x = 4;
% Get Coefficients for one coordinate trajectory
Coeffs = Mark.getCoefficients;
for p = 1:pieces
    % shrink data
    for a = 1:4
        DATA(p, a, x) = myShrink(Coeffs(p, a, x)*pi/180);
    end
    DATA(p, 5:6, x) = TimeVals(p:p + 1);
end

x = 5; 
% Get Coefficients for one coordinate trajectory
Coeffs = Mark.getCoefficients;
for p = 1:pieces
    % shrink data
    for a = 1:4
        DATA(p, a, x) = myShrink(Coeffs(p, a, x));
    end
    DATA(p, 5:6, x) = TimeVals(p:p + 1);
end

    





function val = myShrink(x)
    if abs(x) < 1e-5
        val = 0;
    else
        val = x;
    end
end 






