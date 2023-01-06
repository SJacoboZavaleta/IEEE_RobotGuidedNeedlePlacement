%% Creating a biopsy target population for testing the robot-assited needle placement in Matlab
% To achieve this, a robot-assisted needle placement script (getMultipleData_Sim) was run
% multiple times in Matlab (low-computing cost) in order to understand the theoretical behaviour of our robotic approach under 
% a factorial-method design by using two radiologist variables of manipulation: Breast holder size and needle device type.
%
% Factorial design (8 groups of simulation)
%   Independent variables:
%       Cup size: A,B,C and D. (4 levels)
%       Needle type: FNA(Gauge 20) y CN(Gauge 14) (2 levels)
%
% Randomly sampling a medium-size sample (n1 biopsy targets) from a higher population (N > 50 000 points) was useful 
% to find proportion-based parameters such as p1 (mean of reachable targets).
% 
% % This script pretends to calculate the theoretical target reachability of n1 randomized biopsy targets for 
% all eight simulation groups using the needle insertion algorithm. 
%
% Where:
%   FNA: Fine needle aspiration
%   CN: Core needle
% ------------------------------------
clear, clc
%% Initialization
mainPath = pwd;
robotData = load(strcat(mainPath,'\Simulink_models\Data\robotData.mat'));
pointsData = load(strcat(mainPath,'\Simulink_models\Data\biopsyData.mat'));
holderModel = load(strcat(mainPath,'\Simulink_models\Data\breastHolderModel.mat'));
mdl = 'breastBiopsyMulti';%name simulink model
open_system(mdl,'loadonly');%for open model in second plane

% Defining a finite population of possible biopsy targets
n1 = 1000;

% Groups for simulation variables
listCupSize = {'A' 'B' 'C' 'D'};
listNeedleType = [1,2];%1:FNA, 2:CN(10mm), 3:CN(20mm
%NeedleThrow = 10;
%NeedleDeadSpace = 8;
numCups = 4;
numNeedleDevice = 2;

% Definying initial parameters
offsetPre = 5;%Offset pre-placement

typeTrajInterp = 4;%bspline
typeTrajMode = 1;%joint space

%% Simulation

% *** Variable 1: Breast holder device's size***
for x=1:4   
    % Getting all computed biopsy target positions
    cupSize = listCupSize{x}; 
    if cupSize == 'A'
        biopsyPoint = pointsData.targetDataA;
        holderData = load(strcat(mainPath,'\Simulink_models\Data\dataCupA.mat'));
    elseif cupSize == 'B'
        biopsyPoint = pointsData.targetDataB;
        holderData = load(strcat(mainPath,'\Simulink_models\Data\dataCupB.mat'));
    elseif cupSize == 'C'
        biopsyPoint = pointsData.targetDataC;
        holderData = load(strcat(mainPath,'\Simulink_models\Data\dataCupC.mat'));
    else
        biopsyPoint = pointsData.targetDataD;
        holderData = load(strcat(mainPath,'\Simulink_models\Data\dataCupD.mat'));
    end
    % biopsyPoint is numBiopsyPointsX3
    numBiopsyPoints = length(biopsyPoint);
    
    % Getting a fix randomised sequence
    rng(2021)
    s = lhsdesign(n1,1,'Smooth','on');
    
    pc = biopsyPoint(round(1 + (numBiopsyPoints-1)*s),:)';%1 for first row index    
    
    % Initializing array
    isReachable = zeros(1,n1);

    % **** Variable 2: Biopsy needle type ***
    for y = 1:2
               
        needleType = listNeedleType(y);
        if needleType==1
            needleGauge = 20;
        else
            needleGauge = 14;
        end
        
        % Calling the function getMultipleDataToSIM()
        for i=1:n1
            [~,~,~,~,~,~,~,~,~,~,isReachable(i),~,~,~,~,~,~,~,~,~,~,~] = ...
                getMultipleData_Sim(pc(:,i),cupSize,needleType,needleGauge,offsetPre,robotData,holderData,holderModel);
        end
        
        if needleType == 1
            filename = mainPath+"\Simulink_models\Results\targets"+cupSize+"_FNA"+".mat";
        else
            filename = mainPath+"\Simulink_models\Results\targets"+cupSize+"_CN"+".mat";
        end
        
        save(filename,'isReachable');
    end
end