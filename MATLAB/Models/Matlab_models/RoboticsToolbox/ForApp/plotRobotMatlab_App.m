function ax = plotRobotMatlab_App(robotModel,robotPose,isRobotHome,oldAxes)
%% Plotting robot movement by following end effector trajectory
% INPUTS
% robotModel: Matlab robot model data
% robotPose: Desired configuration for robot placement
% isRobotHome: Is the first robot model definition = TRUE, if not, FALSE
% oldAxes: Previous figure axes. For isRobotHome = FALSE
% OUTPUTS
% ax: plot axis       
% --------------------------------------------------------------
%% 
if isRobotHome
    % Show robot model in figure for first time
    close all;
    fig = figure;
    ax = axes(fig);
    hold(ax,'on');
    robotAxes = show(robotModel,robotPose,'Parent',ax,'Frames','off','PreservePlot',false);
    robotAxes.CameraPositionMode = 'auto';
    xlim(ax,[-1 1]), ylim(ax,[-1 1]), zlim(ax,[0 1.2])%in meters
    title(ax,'Biopsy Robot Home position')
else
    % For a desired trayectory test
    ax = show(robotModel,robotPose,'Parent',oldAxes,'Frames','off','PreservePlot',false);
end

