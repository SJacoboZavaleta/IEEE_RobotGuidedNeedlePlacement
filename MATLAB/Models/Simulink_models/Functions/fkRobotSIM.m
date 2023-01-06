function Ts = fkRobotSIM(q,robotData,eeName)
%% Robot forward kinematic in space reference {s}
% INPUTS
% q = [q1 q2 q3 q4] - joint values for the desired pose of end effector
% robotData: Needed robot data
% eeName: To identify if using end effector frame {e} or link 5 frame {t}     
% OUTPUT
% Ts = transformation matrix SO3 desired
% --------------------------------------------------------------
%% Based on product of exponentials formula (PoE) 
S1 = robotData.S1;
S2 = robotData.S2;
S3 = robotData.S3;
S4 = robotData.S4;
S5 = S4;
if eeName==1
    M = robotData.Mse;
else
    M = robotData.Mst;
end

Ts = MatrixExp6s(S1,q(1))*MatrixExp6s(S2,q(2))*MatrixExp6s(S3,q(3))*MatrixExp6s(S4,q(4))*MatrixExp6s(S5,q(5))*M;
end