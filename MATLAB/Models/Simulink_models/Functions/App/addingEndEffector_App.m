function addingEndEffector_App(robot,modo,Lneedle)
%% Adding a massless coordinate frame for the end effector
% INPUTS
% robot : rigid body tree model for a robot in Matlab
% mode : selecting a tool
% Lneedle : needle length in mm
% NOTE: Units are in meters
%% ------------------------------------

if modo==1
    % Without tool
    eeOffset = 0;
    parentBody = 'link5';     
else
    % With tool
    eeOffset = Lneedle/1000;
    parentBody = 'tool';
end

eeBody = robotics.RigidBody('endEffector');
eeBody.Mass = 0;
eeBody.Inertia = [0 0 0 0 0 0];
setFixedTransform(eeBody.Joint,trvec2tform([0 0 eeOffset]));%From frame 5

try % Update the end effector
    getBody(robot,'endEffector');
    replaceBody(robot,'endEffector',eeBody);
catch % Create and end effector for first time
    addBody(robot,eeBody,parentBody);
end

end