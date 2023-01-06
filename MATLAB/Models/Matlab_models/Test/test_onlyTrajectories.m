%%
clc, close all
robotData = load("D:\nuevaTesis2020\DESARROLLO\MATLAB\ModeloNuevo\Simulink_models\Data\robotData.mat");
appData = load("D:\nuevaTesis2020\DESARROLLO\MATLAB\ModeloNuevo\Simulink_models\Data\AppData.mat");
hs = appData.hs;
ns = appData.ns;
T0 = appData.t0Pre;
Tf = appData.tfPre;

[q,qd,qdd,wayPoints,trajTimes,waypointTimes,jointPointsPath,twp] = getJointTrajectoryPlanner_App(4,hs,ns,T0,Tf,21,robotData);

plotTrajectory(trajTimes,q,qd,qdd,'Names',"Joint " + string(1:5),'WaypointTimes',waypointTimes)