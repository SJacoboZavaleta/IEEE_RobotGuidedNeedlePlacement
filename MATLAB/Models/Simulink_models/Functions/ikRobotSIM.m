function q = ikRobotSIM(pE,n_s,unit,robotData)
%% Robot inverse kinematic in space reference {s]
% INPUTS
% pE: Desidered spatial position of End Effector in mm
% n_s: End-effector spatial direction. Points out from robot.
% unit: if output is in meters or milimeters
% robotData: Needed robot data
% OUTPUT
% q = [q1 q2 q3 q4 q5]' 5x1 - joint values for the desired pose of end effector
% --------------------------------------------------------------
%% Based on a geometrical-algebraical approach

H1 = robotData.H1;
L2 = robotData.L2;
H2 = robotData.H2;
M01 = robotData.M01;
S1 = robotData.S1;
L3 = robotData.L3;

% Joint 1:
q1 = atan2(pE(2),pE(1));
if q1<0 || q1==-pi
    q1 = q1+pi;%(pi-abs(q1));
else
    q1 = q1-pi;%-(pi-abs(q1));
end

Re = robotRotation_App(pE,n_s);
T0e = [Re pE
       0 0 0 1];

T01 = MatrixExp6s(S1,q1)*M01;
T15 = T01\T0e;

% Joint 3:
q3 = atan2(T15(3,3),T15(1,3));

% Joint 4:
q4 = (T15(1,4) + H1 - L3*cos(q3))/cos(q3);

% Joint 2:
q2 = T15(3,4) -L2 - H2 - sin(q3)*(L3+q4);

%Joint 5
q5 = 0;

%To meters unit if needed
if isequal(unit,'m')
    q2 = q2/1000;
    q4 = q4/1000;
end

q = [q1;q2;q3;q4;q5];%q5 is a pasive joint
end