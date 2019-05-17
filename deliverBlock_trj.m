classdef deliverBlock_trj < robot_trj
    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    Combined trajectory for a full pick and place action.   
    -Defines a trajectory from the Loading bay (taken as input)
    -Delivers the block to the drop location determined from the block.
    -Returns to the loading bay
    
    NOTE: All coordinates in column vectors
    
    Constructor inputs:
    loadBay:    loading Bay coordinates (5DOF)
    t0:         start time of trajectory
    ts:         sampling time
    length:     total trajectory time 
    block:      jblock to be delivered
    
    Note: change class to output in meters and radians
    
    
    % Things to improve:
        - potentially change the approach strategy: using shorter approach
        times for the shorter approaches (seccond and third blocks)
        - add optimisation for time of major trajectory segmentss
        - path optimisation, possibly via gradient descent
    
        -Biggest improvements probably come from calibration now...

    %}
    
end