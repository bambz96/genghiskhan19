    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    A simple script that generates trajectories for testing
    
    Currently testing the basic moveBlock_trj
    %}

% Define the loading bay coordinates
LoadingBay = [37.5; 187.5; -3; 90; 0];

% Create the tower
Tower = jTower(200, 0, 0);

% Generate a block to be delivered
FirstBlock = Tower.nextBlock;

SampleTime = 0.1;

Mark = moveBlock_trj(LoadingBay, 0, SampleTime, 5, FirstBlock);

plot(Mark.getTimeseries, Mark.getPosition);  