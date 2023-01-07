function [wayPoints,orientation,waypointTimes,twp] = getPathPlanner_Sim(h_s,n_s,T0,Tf,numWayPoints,robotData)
%% Getting the path planner function to calculate the waypoints for the robot path
% Output data is only usded for Matlab Simulation
% INPUTS
% h_s: Final end effector position before biopsy in {s} frame
% n_s: Needle insertion direction in {s} frame
% T0, TF: Initial and final times for robot trajectory
% numWayPoints: Total Steps for each joint trajectory is an odd number. So, total waypoints are "steps+1". An even number
% robotData: Needed robot data
% OUTPUTS
% wayPoints: End effector positions (3xm size)
% orientation: End effector orientation in z_5 (3xm size)
% waypointTimes: Time series for waypoints
% twp: Time step for each waypoint
% Notes ::
% numPoints_i : Total Steps for each joint trajectory is an odd number
%               So, total waypoints are "steps+1". An even number
%% Inverse Kinematic from desired biopsy target
% Joint limits for end-effector final position
qInitial = zeros(5,1);
qFinal = ikRobotSIM(h_s,n_s,'mm',robotData);

%% Path Planner
% Getting joint values for waypoints
s =  linspace(0,1,numWayPoints);
s = repmat(s,5,1);
qPath = zeros(5,numWayPoints);
for i=1:numWayPoints
    qPath(1:5,i) = qInitial + qFinal.*s(1:5,i);
end

% Getting transform matrix values for waypoints
TPath = zeros(4,4,numWayPoints);
for i=1:numWayPoints
    TPath(:,:,i) = fkRobotSIM(qPath(:,i),robotData,2);
end

% Getting position for waypoints
wayPoints = zeros(3,numWayPoints);
for i=1:numWayPoints
    wayPoints(:,i) = getP(TPath(:,:,i));%in mm
end

% Getting orientation for waypoints
orientation = zeros(3,numWayPoints);
for i=1:numWayPoints
    orientation(:,i) = getZ(TPath(:,:,i));
end
%% Additional parameters
% Waypoints time
% From T0 = 1 and Tf = 4
waypointTimes = linspace(T0,Tf,numWayPoints);
twp = (Tf-T0)/(numWayPoints-1);%cte =  waypointTimes(2)-waypointTimes(1)

% Trajectory sample time
% From T0 = 1 and Tf = 4 with little steps ts
% numWayPoints = 41
% segments = 40
% T=10s --> twp=0.52s ; T=3s --> twp=0.075s

% Note: For using an trapezoidal interpolation
% ts = twp/5; --> twp=0.075 y ts=0.015s
% trajTimes = T0:ts:Tf;

% Rounded values
% robotOrientationZ = round(robotOrientationZ,6);
% robotOrientationR = round(robotOrientationR,6);
% robotWayPoints = round(robotWayPoints,6);
% robotOrientationEuler = round(robotOrientationEuler,6);
% robotOrientationRPY = round(robotOrientationRPY,6);

end