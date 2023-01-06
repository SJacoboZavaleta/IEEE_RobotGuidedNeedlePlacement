function [robotModel,numJoints,homeConfig] = loadingRobot_App(path)
%% For rigid body three model of the robot in Matlab
% ------------------------------------------------------------------
robotData = load(path);
robotModel = robotData.robotModelMAT;
robotModel.DataFormat = 'column';
numJoints = robotModel.NumBodies;
robotModel.Gravity = [0 0 -9.80665];%m/s/s
homeConfig = robotModel.homeConfiguration;

end

