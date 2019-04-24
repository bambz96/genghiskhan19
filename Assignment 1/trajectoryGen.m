%{
Assignment Group 20 
Trajectory Generation using the spline function

Inputs: 
    -x: vector of coordinates of all end points/via points
    -t: corresponding times for each of these points
    -ts: Sample time. 

This trajectory generator asumes zero velocity at the start and end point
hence, a clamped spline is calculated for any input

This is basically just a wrapper function for the spline tool

Notes:
calculate fist for a single trajecotry profile, then develop for multiple
degrees
%}

%So this should work for 
function xx = trajectoryGen(x, t, ts) 
    vi = 0; %initial velocity
    vf = 0; %final veloctiy
    
    tt = t(1):ts:t(end); %timeseries for trajectory 
    
    % initial and final velocitites are input to "clamp" the spline
    xx = spline(t, [vi, x, vf], tt);
end