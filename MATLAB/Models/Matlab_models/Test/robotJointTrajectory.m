% ROBOT TRAJECTORY IN JOINT COORDINATES
% Testing the robot behaviur
% 16/02/22
%% Load and display robot
clc, close
% Uncomment this for first time or a modified version
% robotPath = "D:\nuevaTesis2020\DESARROLLO\SOLIDWORKS\ModeloFinal\2019\URDF\Version2\RobotClinicoSinTool\urdf\RobotClinicoSinTool.urdf";
% robotModelMAT = importrobot(robotPath);
%%% save robotModels.mat robotModelMAT
mainPath = startupGUI;
model = load(strcat(mainPath,'\Simulink_models\Data\robotModels.mat'));
robot = model.robotModelMAT;
robotData = load(strcat(mainPath,'\Simulink_models\Data\robotData.mat'));
M = robotData.M_m;
S = robotData.Slist_m;
%% Adding extra robot parameters
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.80665];
homeConfig = robot.homeConfiguration;    
axes = show(robot);
axes.CameraPositionMode = 'auto';
% Notes:
% getTransform(robot,config,'end_effector','base_link')
% config(4).JointPosition = 0.5;
% showdetails(robot)
% getBody(robot,'link');
% Con el actual udrf format. El efector final se considera el S.C {5}
% pegado al link5!!!
%setFixedTransform(needleBody.Joint,trvec2tform([0 0 offsetNeedle]));%Respecto del origen de link 5
%T = trvec2tform([0.1 0.1 0.2]) * eul2tform([0 0 pi], 'ZYX');
%% Importing needle device
% needlePath = "D:\nuevaTesis2020\DESARROLLO\SOLIDWORKS\ModeloFinal\2019\URDF\Devices\needleTipo1\meshes\needleBody.STL";
% Rneedle_max = 15/2/1000;
% thicknessArmHolder = 3.18/1000;%mm
% thickessNeedleGuide = 2.29/1000;%0.09inch
% Lneedle = 80/1000;%mm
% Lneedle_max = Lneedle + 60/1000;
% needleBody = robotics.RigidBody('tool');
% needleBody.Mass = 0.0099487;%kg
% needleBody.CenterOfMass = [9.2519E-12  4.5308E-09 -0.029936];
% needleBody.Inertia = [3.497E-06 3.497E-06 3.028E-07 -4.9062E-12 -8.6895E-15 1.8157E-17];%[Ixx Iyy Izz Iyz Ixz Ixy] in kg/m2
% addCollision(needleBody,collisionCylinder(Rneedle_max,Lneedle_max))
% offsetNeedle = -(thicknessArmHolder+thickessNeedleGuide);
% T = trvec2tform([0 0 offsetNeedle]);
% setFixedTransform(needleBody.Joint,T);
% addVisual(needleBody, "Mesh",needlePath, T);
% addBody(robot,needleBody,'link5');
%% Add another massless coordinate frame for the end effector

% Remember. C.S of joint 5 or point T
%eeOffset = Lneedle;%with tool

eeOffset = 0;%without tool
eeBody = robotics.RigidBody('endEffector');
eeBody.Mass = 0;
eeBody.Inertia = [0 0 0 0 0 0];
setFixedTransform(eeBody.Joint,trvec2tform([0 0 eeOffset]));%Respecto del origen de link 5

%addBody(robot,eeBody,'tool');%with tool
addBody(robot,eeBody,'link5');%without tool

%axes = show(robot,'Collisions','on','Visuals','off','Frames','on');
%axes = show(robot);
%xlim([-1 1]), ylim([-1 1]), zlim([0 1.2]);
%axes.CameraPositionMode = 'auto';
%hold on;
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
%% Ploting
%exampleHelperPlotWaypoints(pointsPath);
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
% Building a struct data needed for robotics toolbox
% dataStruct = struct('JointName',{'joint1','joint2','joint3','joint4','joint5'});
% qSpace = num2cell(jointPointsPath(i,:));
% [dataStruct.JointPosition] = qSpace{:};
% jointPointsPathStruct(i,:) = dataStruct;

jointPointsPath = zeros(numJoints,numPoints);
%Solving for each trajectory point
for i = 1:numPoints
    jointPointsPath(:,i) = ikRobotSIM(pointsPath(:,i),orientation(:,i),'m',robotData);
end

%% Create a smooth trajectory from the waypoints
trajType = 'cubic';
switch trajType
    case 'trap'
        %Note:time output goes from t0=0 to tF
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
        ctrlpoints = jointPointsPath'; % Can adapt this as needed
        [q,qd,qdd] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
        
    otherwise
        error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
end

%To visualize the trajectory, run the following line
plotTrajectory(trajTimes,q,qd,qdd,'Names',"Joint " + string(1:numJoints),'WaypointTimes',waypointTimes)

%% Visualize robot configurations
% This plot is only for practical views: takes long time consuming
% Using Struct format : 
%config = struct('JointName',{'Jlink1','Jlink2','Jlink3','Jlink4','Jlink5'});
%qSpace = num2cell(q(:,i));% in rad and m
%[config.JointPosition] = qSpace{:};
%eeTform = getTransform(robot,config,'endEffector');
    
for i = 1:N
    % Find Cartesian points for visualization
    eeTform = getTransform(robot,q(:,i),'endEffector');
    %eeTform = fkRobotSIM(q(:,i).*[1 1000 1 1000 1],robotData,2);%but q in m
    
    % Plot the total trajectory
    if plotMode == 1
        eePos = tform2trvec(eeTform);%extraerP(eeTform/1000)
        set(hTraj,'xdata',[hTraj.XData eePos(1)], ...
                  'ydata',[hTraj.YData eePos(2)], ...
                  'zdata',[hTraj.ZData eePos(3)]);
    elseif plotMode == 2
        plotTransforms(tform2trvec(eeTform),tform2quat(eeTform),'FrameSize',0.05);
    end
    
    % Show the robot
    show(robot,q(:,i),'Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(i))])
    drawnow   
end
