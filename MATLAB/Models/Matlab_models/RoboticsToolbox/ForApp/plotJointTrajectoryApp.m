function eePos = plotJointTrajectoryApp(robotModel,q,trajTimes,plotMode,nameEE,ax,hTraj)
%% Plotting end effector trajectory for MATLAB testing
% INPUTS
% q: Joint coordinates or values. (nxm size)
%   n: number of robot joints
%   m: number of waypoints
% trajTimes : Time for waypoints of trayectory. (1xm size)
% nameEE :  Link name for robotModel (MATLAB tree model)
% OUTPUTS
% eePos : End effector position
%% ------------------------------------

eePos = [0 0 0];
N = length(trajTimes);
for i = 1:N
    eeTform = getTransform(robotModel,q(:,i),nameEE);
    
    if plotMode == 1
        eePos = tform2trvec(eeTform);%1x3
        set(hTraj,'xdata',[hTraj.XData eePos(1)], ...
                  'ydata',[hTraj.YData eePos(2)], ...
                  'zdata',[hTraj.ZData eePos(3)]);
    elseif plotMode == 2
        plotTransforms(tform2trvec(eeTform),tform2quat(eeTform),'FrameSize',0.05);
    end
    
    % Showing the movement
    robotAxes = plotRobotMatlab_App(robotModel,q(:,i),false,ax);
    title(robotAxes,['Trajectory at t = ',num2str(trajTimes(i))]);
    drawnow
end

end