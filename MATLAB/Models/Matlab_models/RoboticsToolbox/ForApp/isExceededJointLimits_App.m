function isReachable = isExceededJointLimits_App(h_s,n_s,robotData)
%% Verifying if robot join limits are exceeded. Kinematic limitation
% h_s: End effector final position
% n_s: End effector final direction
% robotData: Needed robot data
% ------------------------------------------------------------------
%% Extracting some data
q_2max = robotData.q_2max;
q_4max = robotData.q_4max;

%% Evaluating Inverse kinematics
q = ikRobotSIM(h_s,n_s,'mm',robotData);

%% Evaluating mechanical joint limits
isReachable = true;
% All joint limit conditions should be met
if abs(q(1))>pi || (q(2)<0 || q(2)>q_2max) || abs(q(3))>pi/2 || (q(4)<0 || q(4)>q_4max)
    isReachable = false;
end

end

