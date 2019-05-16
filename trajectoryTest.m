    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    A simple script that generates trajectories for testing
    
    Currently testing the basic moveBlock_trj

    Now trying to change this to output the required for mat 
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

Greg = grip_trj(LoadingBay, 0);
Mark = moveBlock_trj(LoadingBay, 0, 5, FirstBlock);
Marv = moveBlock_trj(LoadingBay, 0, 5, FirstBlock);

% Greg.plotTrajectories;
% Mark.plotTrajectories;


robot_trj.combineDATA([Greg, Mark], 2)
% 
% DATA1 =  Mark.getDATA;
% DATA2 = Marv.getDATA;
% 
% cat(1, DATA1, DATA2)







