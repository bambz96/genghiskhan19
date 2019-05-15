    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    A simple script that generates trajectories for testing
    
    Currently testing the basic moveBlock_trj

    Now trying to change this to output the required format 
    to send to the robot
    Deffinitely need to improve on this...
    %}

DOF = 5;
pieces = 4;

DATA = zeros(pieces, 6, DOF);

% Define the loading bay coordinates
LoadingBay = [0.0375; 0.1875; -0.003; pi/2; 0];

% Create the tower
Tower = jTower(0.2, 0, 0);

% Generate a block to be delivered
FirstBlock = Tower.nextBlock;

SampleTime = 0.1;

Mark = moveBlock_trj(LoadingBay, 0, SampleTime, 5, FirstBlock);

plot(Mark.getTimeseries, Mark.getPosition);  

TimeVals = Mark.getTime;

% NOTES: for data structure
% rows: pieces
% columns [a3, a2, a1, a0, ts, tf]
% pages: coordinates

for x = 1:DOF
    % Get Coefficients for one coordinate trajectory
    Coeffs = Mark.getCoefficients;
    for p = 1:pieces
        % shrink data
        for a = 1:4
            DATA(p, a, x) = myShrink(Coeffs(p, a, x));
        end
        DATA(p, 5:6, x) = TimeVals(p:p + 1);
    end
end
    





function val = myShrink(x)
    if abs(x) < 1e-5
        val = 0;
    else
        val = x;
    end
end 






