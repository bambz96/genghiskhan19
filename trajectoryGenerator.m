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
LoadingBay = [37.5; 187.5; 20; 90; 0];

% Create the tower
Tower = jTower(200, 0, 0);

% Generate a block to be delivered
FirstBlock = Tower.nextBlock;

SampleTime = 0.1;

Mark = moveBlock_trj(LoadingBay, 0, SampleTime, 2, FirstBlock);
% Mark = release_trj(LoadingBay, 0, SampleTime); 
% Mark = return_trj(LoadingBay, 0, SampleTime, 2, FirstBlock);

plot(Mark.getTimeseries, Mark.getPosition);  

TimeVals = Mark.getTime;

% NOTES: for data structure
% rows: pieces
% columns [a3, a2, a1, a0, ts, tf]
% pages: coordinates

for x = 1:3
    % Get Coefficients for one coordinate trajectory
    Coeffs = Mark.getCoefficients;
    for p = 1:pieces
        % shrink data
        for a = 1:4
            DATA(p, a, x) = myShrink(Coeffs(p, a, x)/1000);
        end
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






