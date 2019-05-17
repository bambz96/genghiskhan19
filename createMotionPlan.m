function [nchunks, chunks] = createMotionPlan()
    %{
        Assignment Gropup 20
        Create a full tower build sequence and output in an appropriate
        form to feed to the Great Jenghis Khan
    
    %}    

    % Set up loading Bay, starting Position and times

    LoadingBay = [0.0375; -0.1875; -0.003; -pi/2; 0];
    InitalPosition = LoadingBay;
    
    % Array storinng all trajectories
    AllTraj = [];
    
    Tower = jTower(0.2, 0, 0);
    
    nblocks = 10;
    
    % Chunks of DATA that each have t0 = 0s, and end with velocity = 0
    % Each chunk will be sent to device separately
    chunks = [];
    
    for cycle = 1:nblocks
        T = 0; % initialise time for every chunk
        
        Block = Tower.nextBlock;
        
        Grip = grip_trj(LoadingBay, T);

        T = T + 0.1;
        Move = moveBlock_trj(LoadingBay, T, 5, Block);

        T = T + 0.1;
        Release = release_trj(Block, T);

        T = T + 0.1;
        Return = return_trj(LoadingBay, T, 5, Block);
        T = T + 0.1;
        
        % changes the block state, so that a new block can be generated
        Block.placeBlock;
        
        AllTraj = [Grip, Move, Release, Return];
        DATA = robot_trj.combineDATA(AllTraj, 4);
        chunks(:,:,:,cycle) = DATA;
        
    end
    
    [~,~,~,nchunks] = size(chunks);

end