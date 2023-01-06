function loadingData_App
%% Saving data in a single file to be uploaded in Simulink workspace before simulation
% For the robot, needle, trajectory and supervisory control of all involved
% elements in the biopsy environment
% ------------------------------------------------------------------
%% For Simulink model
mainPath = pwd;
robotData = load(strcat(mainPath,'\Simulink_models\Data\robotData.mat'));%robot model
breastROM = load(strcat(mainPath,'\Simulink_models\Data\breastROM.mat'));%breast model
breast = breastROM.breast;

% Some dimnensions for robot link 5, needle and end effector. From CAD model
thicknessArmHolder = 3.18;%1/8 inch. In link 4
thickessNeedleGuide = 2.29;%0.09 inch. In link 5
offsetNeedle = thicknessArmHolder+thickessNeedleGuide;%In link 5
offsetL5 = 120;%Length of space in link 3 for needle manipulability.
% It was also considered a semicircular space in link 3 with radius 42mm 

% Some data for trajectory generation 
pointsData = load(strcat(mainPath,'\Simulink_models\Data\robotPathsSIM.mat'));
Ts = pointsData.ts;

% Data from robot-guided biopsy application (GUI)
appData = load(strcat(mainPath,'\Simulink_models\Data\AppData.mat'));

% Needle insertion path
insertionTime = 3;%seconds
numInsertionPoints = 10;%waypoints

% Supervisory logic control
TsModel = Ts; %Equal to the time step for each trajectory pose
%It was enough for a decent simulation

% Model
joint_damping = 0.2;% Revolute joint damping

% Joint error tolerance. 
posTol = [3.21;3.21;3.21]/1000;%From the state of art: 0.54-3.21mm
orientTol = 5*pi/180;%Around 5°

simulationTime = appData.tfPre + insertionTime + 3;%Approx 10s

% Simulated biopsy target - A spherical model
% Type 2 tumor T1: Length < 20mm and Radius~=10mm
Rtarget = 3;%in mm

% Needle material. Based on stainless steel
needle.Rho = 7.93e3;%kg/m^3
needle.Young = 193;%GPa 190-203
needle.Poisson = 0.27;%0.265-0.275%
%needle.Shear = 86;7GPa 4-81
needle.MassCoef = 0.01;%alpha 1/s
needle.StiffCoef = 0.01;%beta s

%% Saving data
pathToSave = strcat(mainPath,'\Simulink_models\Data\simulinkWorkSpace.mat');
save(pathToSave,'-struct', 'robotData');
save(pathToSave,'-struct','appData','-append');
save(pathToSave,'-struct','pointsData','-append');
save(pathToSave,'posTol', 'orientTol', 'insertionTime', 'numInsertionPoints', 'mainPath', 'needle',...
    'joint_damping', 'simulationTime', 'Rtarget','TsModel', ...
    'thicknessArmHolder', 'thickessNeedleGuide', 'offsetNeedle', 'offsetL5', 'robotData','breast','-append');

end