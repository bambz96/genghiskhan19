    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    Testing script to generate the required trajectory to tet the return
    trajectory.

    Deffinitely need to improve on this...
    %}

DOF = 5;
pieces = 4;

DATA = zeros(pieces, 6, DOF);

% Define the loading bay coordinates
LoadingBay = [37.5; 187.5; -3; 90; 0];

% Create the tower
Tower = jTower(200, 0, 0);

% Generate a block (that has been delivered)
FirstBlock = Tower.nextBlock;

SampleTime = 0.1;

Ronda = moveBlock_trj(LoadingBay, 0, SampleTime, 5, FirstBlock);

plot(Ronda.getTimeseries, Ronda.getPosition);  

TimeVals = Ronda.getTime;

% NOTES: for data structure
% rows: pieces
% columns [a3, a2, a1, a0, ts, tf]
% pages: coordinates

for x = 1:DOF
    % Get Coefficients for one coordinate trajectory
    Coeffs = Ronda.getCoefficients;
    for p = 1:pieces
        % shrink data
        for a = 1:4
            DATA(p, a, x) = myShrink(Coeffs(p, a, x)/1000);
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



