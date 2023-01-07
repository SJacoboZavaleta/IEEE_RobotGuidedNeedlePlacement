function eePos = plotTransTrajectoryMatlab(robotModel,Tpath,waypointTimes,plotMode,ax,hTraj)
%% Plotting end effector trajectory for MATLAB testing
% INPUTS
% Tpath: Homegenous transformation matrixes for each robot pose at waypoints
% waypointTimes: Waypoint timesteps
% OUTPUTS
% eePos : End effector position
%% ------------------------------------
eePos = [0 0 0];
N = size(Tpath,3);
for w=1:N-1
    % Get the initial and final transforms and times for the segment
    T0 = Tpath(:,:,w);
    Tf = Tpath(:,:,w+1);
    timeInterval = waypointTimes(w:w+1);
    trajTimes = timeInterval(1):ts:timeInterval(2);%;

    % Find the transforms from trajectory generation
    [T,~,~] = transformtraj(T0,Tf,timeInterval,trajTimes);

    %Trajectory visualization for the segment
    if plotMode == 1
        eePos = tform2trvec(T);%getP
        set(hTraj,'xdata',eePos(:,1),'ydata',eePos(:,2),'zdata',eePos(:,3));
    elseif plotMode == 2
        plotTransforms(tform2trvec(T),tform2quat(T),'FrameSize',0.05);
    end

    for i = 1:numPointsInterm
        % Solve IK
        tgtPose = T(:,:,i);
        q = ikRobot(getP(tgtPose)*1000,getZ(tgtPose),'m');%position in mm

        % Showing the movement
        robotAxes = plotRobotMatlab_App(robotModel,q(:,i),false,ax);
        title(robotAxes,['Trajectory at t = ' num2str(trajTimes(i))])
        drawnow
    end
end

end