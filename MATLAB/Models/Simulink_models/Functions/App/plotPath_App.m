function htraj = plotPath_App(wayPoints,plotMode,ax)
%% Plotting robot path with Matlab
% INPUTS
% wayPoints: 3xnumPoints (in meters)
% plotMode: 0 = None, 1 = show Trajectory line, 2 = Coordinate Frames
% ax: Axis frame for robot model figure
% OUTPUTS
% htraj: Line objet to modify properties
% --------------------------------------------------------------
%% 
if plotMode == 1
    % Getting line properties for first waypoint
    htraj = plot3(ax,wayPoints(1,1),wayPoints(2,1),wayPoints(3,1),'b.-','LineWidth',6);
end

plot3(ax,wayPoints(1,:),wayPoints(2,:),wayPoints(3,:),'r.','LineWidth',3);

end

