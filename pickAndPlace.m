function [Pieces, DATA] = pickAndPlace()
    RIGHT = 1;
    BLOCK = 1;
    
    GRIPTIME = 0.2;
    UNGRIPTIME = 0.2;
    MOVETIME = 4;
    RETURNTIME = 4;
    
    
    
    
    AllTraj = [];
    %{
        Assignment Gropup 20
        Create a full tower build sequence and output in an appropriate
        form to feed to the Great Jenghis Khan
    
    %}
    % Set up loading Bay, starting Position and times

    LoadingBay = [0.0375; -0.1875; -0.003; -pi/2; 0];

    
    % Array storinng all trajectories
    AllTraj = [];
    
    Tower = jTower(0.2, -0.05, 0);
    
    T = 0; % initialise time 
    

    Block = Tower.nextBlock;
    
    % Iterate to desired block
    for i = 2:BLOCK
        Block.placeBlock;
        Block = Tower.nextBlock;
    end

    close all;
    
    Grip = grip_trj(LoadingBay, T, GRIPTIME);
    Grip.plotTrajectories;
    T = T + GRIPTIME;
    Move = moveBlock_trj(LoadingBay, T, MOVETIME, Block);
    Move.plotTrajectories;
    T = T + MOVETIME;
    Release = release_trj(Block, T, UNGRIPTIME);
    Release.plotTrajectories;
    T = T + UNGRIPTIME;
    Return = return_trj(LoadingBay, T, RETURNTIME, Block);
    Return.plotTrajectories;
    T = T + RETURNTIME;

    Block.placeBlock;


    AllTraj = [AllTraj, Grip, Move, Release, Return];
    
        
    
    DATA = robot_trj.combineDATA(AllTraj, 4);
    [Pieces, ~, ~] = size(DATA);
    
end