function [q,dq,ddq,wayPoints,trajTimes,waypointTimes,jointPointsPath,twp] = getJointTrajectoryPlanner_App(trajType,h_s,n_s,T0,Tf,numPoints,numJoints,robotData)
%% Getting robot trajectories in joint space
% INPUTS
% trajType: Trajectory type such as cubic, quintic, trapezoidal, ...
% h_s: End-effector final position
% n_s: End-effector final direction
% T0: Initial time
% Tf: Final time
% numPoints: Number of waypoints for each joint path
% robotData : Neede robot data
% OUTPUTS:
% q,dq,ddq: joint position and its derivatives in meters
% wayPoints: in meters
% trajTimes: timeline for each interpolation point
% waypointTimes: timeline for each waypoint
% jointPointsPath: joint values for each waypoint
% twp: Time step for each waypoint
% --------------------------------------------------------------------------
%% Path Planning
[wayPoints,orientationZ,~,trajTimes,waypointTimes,twp] = getPathPlanner_App(h_s,n_s,T0,Tf,numPoints,robotData);

%% Trajectory Planning
N = length(trajTimes); 

% 1. Perform Inverse Kinematics
jointPointsPath = zeros(numJoints,numPoints);%joint values
% Solving for each trajectory point
for i = 1:numPoints
    jointPointsPath(:,i) = ikRobotSIM(wayPoints(:,i),orientationZ(:,i),'m',robotData);%in meters
end

% 2. Get trajectory parameters in radians and meters for matlab simulation
endTime = 0;
accelTime = 0;
waypointVels = zeros(numJoints,numPoints);
waypointAccels = waypointVels;
switch trajType
    case 1% Trapezoidal
        % Boundary conditions
        endTime = twp;% tF
        accelTime = endTime/4;% tA
        % Where:
        % v = (qF - q0)/(tF - tA);
        % a = v/tA;
        % 'PeakVelocity' :v
        % 'Acceleration' :a
        % 'EndTime' : tF or ts
        % 'AccelTime': tA; if v=a*tA
    case 2% Cubic
        % For 'VelocityBoundaryCondition'
        waypointVels = zeros(numJoints,numPoints);
        %waypointVels = [diff(jointPointsPath,1,2)/twp, zeros(5,1)];
        %waypointVels = cat(1,directionVelocity1,directionVelocity2(2:end,:),...
        %                    directionVelocity3(2:end,:),directionVelocity4(2:end,:));
    case 3% Quintic
        % For 'VelocityBoundaryCondition'
        waypointVels = zeros(numJoints,numPoints);
        %waypointVels = [diff(jointPointsPath,1,2)/twp, zeros(5,1)];
        %waypointVels = cat(1,directionVelocity1,directionVelocity2(2:end,:),...
        %                    directionVelocity3(2:end,:),directionVelocity4(2:end,:));
        % For 'AccelerationBoundaryCondition'
        waypointAccels = zeros(numJoints,numPoints);
    otherwise% Bspline
        % No parameters
end

% 3. Create a smooth trajectory from the waypoints
switch trajType
    case 1 % Trapezoidal
        [q,dq,ddq] = trapveltraj(jointPointsPath,N, ...
                        'AccelTime',accelTime, ... 
                        'EndTime',endTime);             
    case 2% Cubic
        [q,dq,ddq] = cubicpolytraj(jointPointsPath,waypointTimes,trajTimes,...
                        'VelocityBoundaryCondition',waypointVels);        
    case 3% Quintic
        [q,dq,ddq] = quinticpolytraj(jointPointsPath,waypointTimes,trajTimes,...
                        'VelocityBoundaryCondition',waypointVels, ...
                        'AccelerationBoundaryCondition',waypointAccels);   
    case 4% Bspline
        [q,dq,ddq] = bsplinepolytraj(jointPointsPath,waypointTimes([1 end]),trajTimes);
        
    otherwise
        error('Invalid trajectory type! Use ''trapezoidal'', ''cubic'', ''quintic'', or ''bspline''');
end

% 4. Exporting waypoint vectors
wayPoints  = wayPoints/1000;% in meters for matlab simulation      
end