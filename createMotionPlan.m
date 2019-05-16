function DATA = createMotionPlan()
    %{
        Assignment Gropup 20
        Create a full tower build sequence and output in an appropriate
        form to feed to the Great Jenghis Khan
    
    %}
    % Set up loading Bay, starting Position and times

    LoadingBay = [0.0375; 0.1875; -0.003; pi/2; 0];
    InitalPosition = LoadingBay;
    
    % Array storinng all trajectories
    AllTraj = [];
    
    Tower = jTower(0.2, 0, 0);
    
    T = 0; % initialise time 
    
    for cycle = 1:Tower.NumBlocks
        Block = Tower.nextBlock;
        
        Grip = grip_trj(LoadingBay, T);

        T = T + 1;
        Move = moveBlock_trj(LoadingBay, T, 5, Block);

        T = T + 5;
        Release = release_trj(Block, T);

        T = T + 1;
        Return = return_trj(LoadingBay, T, 5, Block);
        T = T + 5;
        
        % changes the block state, so that a new block can be generated
        Block.placeBlock;
        
        
        AllTraj = [AllTraj, Grip, Move, Release, Return];
        
    end
    
    DATA = robot_trj.combineDATA(AllTraj, 4*Tower.NumBlocks);
    
end