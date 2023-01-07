function [isReachable,isJointLimit] = isReacheablePoint_Sim(h_s,n_s,collision2,iCollision1,collision3,robotData)
%% Verifying the reachability of biopsy target
% INPUTS
% h_s: Biopsy target position in frame {s}
% n_s: Needle insertion direction in frame {s}
% collision2, collision3: type 2 and 3 collisions
% iCollision1: inevitable type 1 collision
% robotData: 
% OUTPUTS
% isReachable: if target can be reached
% isJointLimit: if any robot joint limit was exceeded
% --------------------------------------------------------------
%% Getting needed data
q_2max = robotData.q_2max;
q_4max = robotData.q_4max;

%% Evaluating Inverse kinematics
q = ikRobotSIM(h_s,n_s,'mm',robotData);

%% Evaluating mechanical joint limits
isJointLimit = false;
if abs(q(1))>pi || (q(2)<0 || q(2)>q_2max) || abs(q(3))>pi/2 || (q(4)<0 || q(4)>q_4max)
    isJointLimit = true;
end

% If there's no collisition with holder main body and the robot can reach
% the preposition without forcing limit joints so the target is reachable
isReachable =  ~isJointLimit && ~collision2 && ~iCollision1 && ~collision3;
% ~collision2 : it means the needle doesn't pass through the required lead
%               so it could be in collision with breast holder main body
% iCollision1 : it means that the target is not reachable by
%               type 1 collision, because of maximal angle range of insertion is in collision 
% ~collision3 : there's no breast chest damage
end