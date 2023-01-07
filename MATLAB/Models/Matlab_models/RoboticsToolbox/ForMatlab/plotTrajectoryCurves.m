function plotTrajectoryCurves(trajTimes,q,dq,ddq,numJoints,waypointTimes)
%% Plotting joint trajectory curves in joint space
% INPUTS
% trajTimes: 
% q: robot position (nxm size)
% dq: robot velocity (nxm size)
% ddq: robot acceleration (nxm size)
% numJoints: n
% waypointTimes: (1xm size)
% OUTPUTS
% eePos : End effector position
%% ------------------------------------
plotTrajectory(trajTimes,q,dq,ddq,'Names',"Joint " + string(1:numJoints),'WaypointTimes',waypointTimes)
end

