%% ROBOT TRAJECTORY IN JOINT COORDINATES
% Testing the robot behavior. Using internal Robotics toolbox of Matlab
clear, clc, close all
%% Load and display robot

%**********
% Uncomment this for first time or a modified version
% robotPath = "\urdf\robotModel.urdf";
% robotModelMAT = importrobot(robotPath);
% save robotModels.mat robotModelMAT
%**********

mainPath = getRootDirectory;
robotData = load(strcat(mainPath,'\Simulink_models\Data\robotData.mat'));

M = robotData.M_m;% Robot home position in meters
S = robotData.Slist_m;% All joint screw axis in meters

model = load(strcat(mainPath,'\Simulink_models\Data\robotModels.mat'));
robot = model.robotModelMAT;
showdetails(robot)
%% Adding extra robot parameters
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.80665];%m/s/s
homeConfig = robot.homeConfiguration;    
axes = show(robot);
axes.CameraPositionMode = 'auto';

%% Importing needle device

importNeedleDevice = true;

if importNeedleDevice
    needlePath = strcat(mainPath,"\CAD_models\Devices\needle1\meshes\needleBody.STL");
    Rneedle_max = 15/2/1000;%mm
    thicknessArmHolder = 3.18/1000;%mm
    thickessNeedleGuide = 2.29/1000;%0.09inch
    Lneedle = 80/1000;%mm
    Lneedle_max = Lneedle + 60/1000;
    
    needleBody = robotics.RigidBody('tool');
    needleBody.Mass = 0.0099487;%kg
    needleBody.CenterOfMass = [9.2519E-12  4.5308E-09 -0.029936];
    needleBody.Inertia = [3.497E-06 3.497E-06 3.028E-07 -4.9062E-12 -8.6895E-15 1.8157E-17];%[Ixx Iyy Izz Iyz Ixz Ixy] in kg/m2
    
    addCollision(needleBody,collisionCylinder(Rneedle_max,Lneedle_max))
    
    needlePosition = -(thicknessArmHolder+thickessNeedleGuide);
    T = trvec2tform([0 0 needlePosition]);
    setFixedTransform(needleBody.Joint,T);
    
    addVisual(needleBody,"Mesh",needlePath, T);
    addBody(robot,needleBody,'link5');

    % For adding a massless coordinate frame for the end effector
    eeOffset = Lneedle;%Using a tool (in needle tip or origin of {e})
    
    newBodyName = 'tool';
else
    % For adding a massless coordinate frame for the end effector
    eeOffset = 0;%No using tool (in needle device base in frame {5} or frame {t})
    newBodyName = 'link5';
end

% For adding a massless coordinate frame for the end effector
eeBody = robotics.RigidBody('endEffector');
eeBody.Mass = 0;
eeBody.Inertia = [0 0 0 0 0 0];
setFixedTransform(eeBody.Joint,trvec2tform([0 0 eeOffset]));%From frame {5}

addBody(robot,eeBody,newBodyName);

showdetails(robot)%Showing new robot tree structure

%Plotting the robot
%axes = show(robot,'Collisions','on','Visuals','off','Frames','on');
axes = show(robot);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2]);
axes.CameraPositionMode = 'auto';
hold on;

%% Create a set of desired waypoints
% ****** Using first original path planning **********
% pointsData = load(strcat(mainPath,'\Simulink_models\Data\robotPathsSIM.mat'));
% wayPoints = pointsData.robotWayPoints;
% pointsPath = wayPoints/1000;%in meters.
% orientation = pointsData.robotOrientationZ;
% orientationRPY = pointsData.robotOrientationRPY;
% numJoints = 5;
% numPoints= pointsData.numPoints;
% trajTimes = pointsData.trajTimes;
% waypointTimes = pointsData.waypointTimes;
% waypointVels = pointsData.waypointVels;
% waypointAccels = pointsData.waypointAccels;
% endTime = pointsData.endTime;
% accelTime = pointsData.accelTime;
% ts = pointsData.ts;
% twp = pointsData.twp;
% N = length(trajTimes); 
 
% ****** Using second approach **********
pointsData = load(strcat(mainPath,'\Matlab_models\Data\newTrajectory.mat'));
% Data 'newTrajectory.mat' calculated in kinematicTest.mlx
wayPoints = pointsData.deltaP;%in m
pointsPath = wayPoints*1000;
wayPointsT = pointsData.deltaT;%in m
waypointTimes = pointsData.timePoints;
numPoints = pointsData.numPoints;
numJoints = 5;
orientation = zeros(3,numPoints);
for i=1:numPoints
    orientation(:,i) = extraerZ(wayPointsT(:,:,i));
end
ts = pointsData.ts;
twp = pointsData.twp;
trajTimes = pointsData.t;%in m
N = length(trajTimes); 
waypointVels = zeros(numJoints,numPoints);

%% Plotting waypoints
plotMode = 1; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
axes = show(robot,homeConfig,'Frames','off','PreservePlot',false);
axes.CameraPositionMode = 'auto';
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on;

if plotMode == 1
    hTraj = plot3(wayPoints(1,1),wayPoints(2,1),wayPoints(3,1),'b.-');%start point
end

plot3(wayPoints(1,:),wayPoints(2,:),wayPoints(3,:),'ro','LineWidth',2);
%% Perform Inverse Kinematics
jointPointsPath = zeros(numJoints,numPoints);
%Solving for each trajectory point
for i = 1:numPoints
    jointPointsPath(:,i) = ikRobotSIM(pointsPath(:,i),orientation(:,i),'m',robotData);
end

%% Create a smooth joint trajectory for the waypoints
%Note:Time output goes from t0=0 to tF
trajType = 'cubic';
switch trajType
    case 'trap'
        [q,qd,qdd,tsample] = trapveltraj(jointPointsPath',N, ...
                        'AccelTime',accelTime, ... 
                        'EndTime',endTime);
                        %'PeakVelocity', 'Acceleration',
                        %'EndTime' — Duration of each trajectory segment
                        %'AccelTime' — Duration of acceleration phase of velocity profile         
    case 'cubic'
        [q,qd,qdd] = cubicpolytraj(jointPointsPath,waypointTimes,trajTimes,...
                        'VelocityBoundaryCondition',waypointVels);        
    case 'quintic'
        [q,qd,qdd] = quinticpolytraj(jointPointsPath',waypointTimes,trajTimes,...
                        'VelocityBoundaryCondition',waypointVels', ...
                        'AccelerationBoundaryCondition',waypointAccels);   
    case 'bspline'
        [q,qd,qdd] = bsplinepolytraj(jointPointsPath',waypointTimes([1 end]),trajTimes);
        
    otherwise
        error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
end

%To visualize the trajectories
plotTrajectory(trajTimes,q,qd,qdd,'Names',"Joint " + string(1:numJoints),'WaypointTimes',waypointTimes)

%% Visualize robot configurations
% This plot is only for practical views: takes long time consuming
    
for i = 1:N
    % Find Cartesian points for visualization
    eeTform = getTransform(robot,q(:,i),'endEffector');
    %eeTform = fkRobotSIM(q(:,i).*[1 1000 1 1000 1],robotData,2);
    
    % Plot the total trajectory
    if plotMode == 1
        eePos = tform2trvec(eeTform);%extraerP(eeTform/1000)
        set(hTraj,'xdata',[hTraj.XData eePos(1)], ...
                  'ydata',[hTraj.YData eePos(2)], ...
                  'zdata',[hTraj.ZData eePos(3)]);
    elseif plotMode == 2
        plotTransforms(tform2trvec(eeTform),tform2quat(eeTform),'FrameSize',0.05);
    end
    
    % Showing the movement
    show(robot,q(:,i),'Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(i))])
    drawnow   
end
