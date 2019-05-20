function [nchunks, chunks] = createMotionPlan(x,y,theta,loadSide, speed)
    %{
        Assignment Gropup 20
        Create a full tower build sequence and output in an appropriate
        form to feed to the Great Jenghis Khan
    
    %}    
    %% Set up loading Bay, starting Position and Testing paramters
    STARTBLOCK = 1;
    NBLOCKS = 54; % Max 54
    
    GRIPTIME = 0.1;
    UNGRIPTIME = 0.2;
    FASTMOVE = 1.5;
    FASTRETURN = 1.2;
    SLOWMODE = 3;
    T_INCREMENT = 0.05;
    
    SPEED = 1.0;    % variable between 0 and 1 (1 is max speed)
    
    MoveTime = SLOWMODE - SPEED*(SLOWMODE - FASTMOVE);
    ReturnTime = SLOWMODE - SPEED*(SLOWMODE - FASTRETURN);
    
    
    
    
    if strcmp(loadSide,'RIGHT') 
        BuildSide = 1;
        LoadingBay = [0.040; -0.1875; -0.005; -pi/2; 0];
    elseif strcmp(loadSide,'LEFT')
        BuildSide = 0; 
        LoadingBay = [-0.000; 0.1875; -0.005; pi/2; 0];
    end
    
    if  strcmp(speed,'Accurate')
        % To implement: set to slow mode
    elseif strcmp(speed,'Balanced')
        % To implement: set to balanced mode
    elseif strcmp(speed,'Full Jenghis')
        % To implement: set to fast mode
    end


    

    
%     Tower = jTower(0.2, -0.05, 0, RIGHT);
    Tower = jTower(x/1000, y/1000, theta*pi/180, 1);
    
        
    
    %% Iterate to correct starting block
    for i = 2:STARTBLOCK
        Block = Tower.nextBlock;
        Block.placeBlock;
    end
    
    %% Create Trajectories
    % Chunks of DATA that each have t0 = 0s, and end with velocity = 0
    % Each chunk will be sent to device separately
    chunks = [];
    
    for cycle = STARTBLOCK:NBLOCKS
        T = 0; % initialise time for every chunk
        
        Block = Tower.nextBlock;
        
        Grip = grip_trj(LoadingBay, T, GRIPTIME);
        T = T + GRIPTIME;
        
        Move = moveBlock_trj(LoadingBay, T, MoveTime, Block);
        T = T + MoveTime;
        
        Release = release_trj(Block, T, UNGRIPTIME);
        T = T + UNGRIPTIME;
        
        Return = return_trj(LoadingBay, T, ReturnTime, Block);
        T = T + ReturnTime;
        
        % changes the block state, so that a new block can be generated
        Block.placeBlock;
        
        AllTraj = [Grip, Move, Release, Return];
        DATA = robot_trj.combineDATA(AllTraj, 4);
        chunks(:,:,:,cycle) = DATA;
        
%         if mod(cycle, 3) == 0
%             MoveTime = MoveTime + T_INCREMENT;
%             ReturnTime = ReturnTime + T_INCREMENT;
%         end
        
    end
    
    [~,~,~,nchunks] = size(chunks);

end
