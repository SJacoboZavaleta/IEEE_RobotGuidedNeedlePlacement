function [TPath,waypointTimes,twp] = getTransTrajectoryPlannerApp(h_s,n_s,T0,Tf,numPoints,robotData)
%% Getting robot trajectories in transformation coordinates
[~,~,TPath,~,waypointTimes,twp] = getPathPlanner_App(h_s,n_s,T0,Tf,numPoints,robotData);

% Using units in meters
N = size(TPath,3);
for i=1:N
    TPath(1:3,4,i) = getP(TPath(:,:,i))/1000;
end