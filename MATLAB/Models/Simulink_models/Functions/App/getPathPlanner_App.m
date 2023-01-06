function [wayPoints,orientation,trajTimes,waypointTimes,twp] = getPathPlanner_App(h_s,n_s,T0,Tf,numPoints,numJoints,robotData)
%% Getting the path planner function to calculate the waypoints for the robot path
% Output data is only usded for Matlab Simulation
% INPUTS
% h_s: Final end effector position before biopsy in {s} frame
% n_s: Needle insertion direction in {s} frame
% T0, TF: Initial and final times for robot trajectory
% numPoints: Total Steps for each joint trajectory is an odd number. So, total waypoints are "steps+1". An even number
% robotData: Needed robot data
% OUTPUTS
% wayPoints: End effector positions (3xm size)
% orientation: End effector orientation in z_5 (3xm size)
% trajTimes: Time series for trajectories
% waypointTimes: Time series for waypoints
% twp: Time step for each waypoint
%% Inverse Kinematic from desired biopsy target
% Joint limits for end-effector final position
qInitial = zeros(5,1);
qFinal = ikRobotSIM(h_s,n_s,'mm',robotData);

%% Path Planner
% Getting joint values for waypoints
s =  linspace(0,1,numPoints);
s = repmat(s,5,1);
qPath = qInitial + qFinal.*s;%3XnumWayPoints in meters

% Getting transform matrix values for waypoints
TPath = zeros(4,4,numPoints);
for i=1:numPoints
    TPath(:,:,i) = fkRobotSIM(qPath(:,i),robotData,2);
end

% Getting position for waypoints
wayPoints = zeros(3,numPoints);
for i=1:numPoints
    wayPoints(:,i) = extraerP(TPath(:,:,i));%in mm
end

% Getting orientation for waypoints
orientation = zeros(3,numPoints);
for i=1:numPoints
    orientation(:,i) = extraerZ(TPath(:,:,i));
end

%% Waypoints time
% From T0 = 1 and Tf = 4
waypointTimes = linspace(T0,Tf,numPoints);
twp = (Tf-T0)/(numPoints-1);%cte =  waypointTimes(2)-waypointTimes(1)

% Trajectory sample time
% From T0 = 1 and Tf = 4 with little steps ts
% numPoints = 21
% segments = 20 = N1
% T=3s --> twp=0.15s

% Note: For using an trapezoidal interpolation
% ts = twp/20; --> twp=0.15 y ts=0.001s
% ts = twp/(N1*N2)=0.01; N1=20(segmentos de waypoints) y N2=15(segmentos para trayectorias dentro de cada waypointTime);
ts = twp/15;
trajTimes = T0:ts:Tf;

%% Saving data in a new file for Simulink simulation
pathToSave = pwd;
filename = strcat(pathToSave,'\Simulink_models\Data\robotPathsSIM.mat');

save(filename,'ts', 'numPoints', 'numJoints');

end