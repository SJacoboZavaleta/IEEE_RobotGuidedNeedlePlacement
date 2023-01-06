%% Getting data for multiple randomized simulation
% Automated script to simulate robot-assisted needle placement in a
% Simulink environment. Based on script "getMultipleData_Sim.m" and App 'breastBiopsyMulti.slx'
% 
% Basic settings for a particular case:
%   cupSize = 'A'
%   needleType = 1(FNA), 2(CN)
%   needleGauge = 11, 14, 18, 20, 21
%   needleThrow = 20
%   needleDeadSpace = 8 mm
%   offsetPre = 5 mm
%---------------------------------
clear, clc
%% Initialization
mainPath = pwd;
robotData = load(strcat(mainPath,'\Simulink_models\Data\robotData.mat'));
pointsData = load(strcat(mainPath,'\Simulink_models\Data\biopsyData.mat'));
holderModel = load(strcat(mainPath,'\Simulink_models\Data\breastHolderModel.mat'));
mdl = 'breastBiopsyMulti';% name of simulink model
open_system(mdl,'loadonly');%for opening model in second plane

% Groups for simulation variables
listCupSize = {'A' 'B' 'C' 'D'};
listNeedleType = [1,2];%1:FNA, 2:CN(10mm), 3:CN(20mm)
numCups = 4;% number of breast holder sizes
numNeedleDevice = 2;% number of needle device types

% Definying initial parameters
offsetPre = 5;% Offset distance between the end effector and breast holder after the preplacement

typeTrajInterp = 4;%bspline
typeTrajMode = 1;%In joint space

%Rtarget = 3;% Target radius of 6mm length (cancerous lesions)

% From computeTargetsSampling.m
nSamples = [160 137 98 87];
%% Simulation

% *** Variable 1: Breast Holder device ***
for x=1:1
    % Getting sample number
    %numTargets = 1;
    numTargets = nSamples(x);
    
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
    rng(2021)%muestra obtenida con otro valor 2022
    s = lhsdesign(numTargets,1,'Smooth','on');
    
    pc = biopsyPoint(round(1 + (numBiopsyPoints-1)*s),:)';%1 for first row index     
    %pc(:,1) = [1.24,-48.28,1087]';% pc is 3XnumBiopsyPoints
    
    %Zero Initial Values
    ps = zeros(3,numTargets);
    hs = zeros(3,numTargets);
    ns = zeros(3,numTargets);
    collision1 = zeros(1,numTargets);
    iCollision1 = collision1;
    collision2 = collision1;
    pCollision2 = collision1;
    collision3 = collision1;
    collision4 = collision1;
    isJointLimit = collision1;
    isReachable = collision1;
    numLeadToOpen = collision1;
    numLeadToCol = collision1;
    dInsertion = collision1;
    needleLength = collision1;
    rNeedle = collision1;
    needleThrow = collision1;
    needleDeadSpace = collision1;
    needleDeviceLength = collision1;
    wasRotated = collision1;
    mechUpperDelta=collision1;
    realUpperDelta=collision1;
    mechLowerDelta=collision1;
    realLowerDelta=collision1;
    
    % *** Variable 2: Biopsy needle type ***
    for y = 1:2
        needleType = listNeedleType(y);
        if needleType==1
            needleGauge = 20;
        else
            needleGauge = 14;
        end
        
        % Calling the function getMultipleDataToSIM()
        %i=1:numTargets
        for i=1:numTargets
            [ps(:,i),hs(:,i),ns(:,i),collision1(i),iCollision1(i),pCollision2(i),collision2(i),wasRotated(i),collision3(i),isJointLimit(i),isReachable(i),...
                numLeadToOpen(i),dInsertion(:,i),needleLength(i),rNeedle(i),needleThrow(i),needleDeadSpace(i),needleDeviceLength(i),...
                mechUpperDelta(i),realUpperDelta(i),mechLowerDelta(i),realLowerDelta(i)] = ...
                getMultipleData_Sim(pc(:,i),cupSize,needleType,needleGauge,offsetPre,robotData,holderData,holderModel);
        end
        
        %% Preparing simulink model
        for i = 1:numTargets
            simIn(i) = Simulink.SimulationInput(mdl);
            simIn(i) = setVariable(simIn(i),'robotData',robotData,'Workspace',mdl);
            simIn(i) = setVariable(simIn(i),'ps',ps(:,i),'Workspace',mdl);
            simIn(i) = setVariable(simIn(i),'hs',hs(:,i),'Workspace',mdl);
            simIn(i) = setVariable(simIn(i),'ns',ns(:,i),'Workspace',mdl);
            simIn(i) = setVariable(simIn(i),'sizeHolderCup',cupSize,'Workspace',mdl);
            simIn(i) = setVariable(simIn(i),'needleGauge',needleGauge,'Workspace',mdl);
            simIn(i) = setVariable(simIn(i),'needleType',needleType,'Workspace',mdl);
            simIn(i) = setVariable(simIn(i),'reachablePoint',isReachable(i),'Workspace',mdl);
            simIn(i) = setVariable(simIn(i),'numHolderLead',numLeadToOpen(i),'Workspace',mdl);
            simIn(i) = setVariable(simIn(i),'Rneedle',rNeedle(i),'Workspace',mdl);
            simIn(i) = setVariable(simIn(i),'needleThrow',needleThrow(i),'Workspace',mdl);
            simIn(i) = setVariable(simIn(i),'needleLength',needleLength(i),'Workspace',mdl);
            simIn(i) = setVariable(simIn(i),'needleDeadSpace',needleDeadSpace(i),'Workspace',mdl);
            simIn(i) = setVariable(simIn(i),'needleDeviceLength',needleDeviceLength(i),'Workspace',mdl);
            simIn(i) = setVariable(simIn(i),'typeTrajMode',typeTrajMode,'Workspace',mdl);%
            simIn(i) = setVariable(simIn(i),'typeTrajInterp',typeTrajInterp,'Workspace',mdl);%
        end
        simOutputs = sim(simIn);
                
        %% Simulation constriction
        pc_n = round(pc,2);
        ps_n = round(ps,2);
        hs_n = round(hs,2);
        ns_n = round(ns,2);
        for i=1:numTargets
            aux0 = string(pc_n(:,i));
            pc_c(i) = "["+aux0(1)+","+aux0(2)+","+aux0(3)+"]";
            aux1 = string(ps_n(:,i));
            ps_s(i) = "["+aux1(1)+","+aux1(2)+","+aux1(3)+"]";
            aux2 = string(hs_n(:,i));
            hs_s(i) = "["+aux2(1)+","+aux2(2)+","+aux2(3)+"]";
            aux3 = string(ns_n(:,i));
            ns_s(i) = "["+aux3(1)+","+aux3(2)+","+aux3(3)+"]";
        end
         
%         % Table 1: Just for reporting
%         cupSizes = ["A,B,C,D","A,B,C,D"];
%         needleGaugues = [20,16];
%         deviceLengths = [60,83];
%         angleVars = [0,0];
%         offsetPres = [10,10];
%         longitudThrows = [0,10];
%         longitudEspacioMuerto = [0,8];
%         caso1 = [cupSizes(1) needleGaugues(1) angleVars(1) offsetPres(1) longitudThrows(1) longitudEspacioMuerto(1) deviceLengths(1)];
%         caso2 = [cupSizes(2) needleGaugues(2) angleVars(2) offsetPres(2) longitudThrows(2) longitudEspacioMuerto(2) deviceLengths(2)];
%         t1 = table(caso1',caso2','VariableNames',{'FNAB','CNB'},'RowNames',...
%             {'Copa de mama' 'Calibre Aguja (Gauge)' 'Angulo insercion (0°)' 'Offset Pre-precement (mm)' 'Longitud Throw (mm)' 'Longitud dead space (mm)' 'Longitud Dispotivo Biopsia (mm)'});
%         %disp(t1)
        
        %% Biopsy Prediction
        % Table 2
        t2 = table(pc_c',ps_s',hs_s',ns_s',collision1',iCollision1',pCollision2',collision2',wasRotated',collision3',collision4',isJointLimit',isReachable',numLeadToOpen',...
            dInsertion',needleLength',realUpperDelta',realLowerDelta',mechUpperDelta',mechLowerDelta',...
            'VariableNames',{'pc' 'ps' 'hs' 'ns' 'Colision1' 'colision1Inev' 'colision2Old' 'colision2' 'rotacion' 'colision3' 'colision4' 'jointLimits' ...
            'alcanzabilidad' 'tapaInsercion' 'distanciaInsercion' 'longitudAguja' 'realUpperDelta' 'realLowerDelta' 'mechUpperDelta' 'mechLowerDelta'});
        %disp(t2)
        
        %% Pre-placement and Post-placement data
        % Table 3, 4 y 5
        numTest = 1:1:numTargets;
        errorPosPre = zeros(numTargets,1);
        errorPosPost = errorPosPre;
        errorOrientPre = errorPosPre;
        errorOrientPost = errorPosPre;
        errorDeviceLength = errorPosPre;
        isSuccessful = errorPosPre;
        pNeedle = zeros(numTargets,3);
        pT = pNeedle;
        zEE = pNeedle;
        zNeedle = pNeedle;
         
        for i=1:numTargets
            if isReachable(i)==1
                data = simOutputs(1,i).logsout;
                % Organized by vector length.
                % *** All outputs from supervisory control have TsModel clock time ***
                % Preplacement stage
                timeSeries = data.get('ePosPre').Values.Time;
                aux = find(timeSeries>4.6);%End of measurements -> 5.1 
                errorPosPre(i) = data.get('ePosPre').Values.Data(aux(1),:)*1000;
                errorOrientPre(i) = data.get('eOrientPre').Values.Data(aux(1),:);
                % postplacement stage
                timeSeries = data.get('ePosPost').Values.Time;
                aux = find(timeSeries>8.85);%End of measurements --> 9.3
                errorPosPost(i) = data.get('ePosPost').Values.Data(aux(1),:)*1000;
                errorOrientPost(i) = data.get('eOrientPost').Values.Data(aux(1),:);
                
                isSuccessful(i) = data.get('biopsySuccess').Values.Data(end);
                
                %*** Outputs from simscape ***
                pT(i,:) = data.get('pT_c').Values.Data(end,:)*1000;
                zEE(i,:) = data.get('zEE_m').Values.Data(:,:,end)';
                timeSeries = data.get('pNeedle_m').Values.Time;
                aux = find(timeSeries>8.85);%End of measurements
                pNeedle(i,:) = data.get('pNeedle_m').Values.Data(aux(1),:)*1000;
                zNeedle(i,:) = data.get('zNeedle_m').Values.Data(:,:,aux(1))';  
            end
        end
        
        for i=1:numTargets
            aux1 = string(round(pT(i,:),2));
            pT_s(i) = "["+aux1(1)+","+aux1(2)+","+aux1(3)+"]";
            aux2 = string(round(zEE(i,:),2));
            zEE_s(i) = "["+aux2(1)+","+aux2(2)+","+aux2(3)+"]";
            aux3 = string(round(zNeedle(i,:),2));
            zNeedle_s(i) = "["+aux3(1)+","+aux3(2)+","+aux3(3)+"]";
            aux4 = string(round(pNeedle(i,:),2));
            pNeedle_s(i) = "["+aux4(1)+","+aux4(2)+","+aux4(3)+"]";
        end
        
        t3 = table(numTest',pT_s',hs_s',round(errorPosPre,2),pNeedle_s',ps_s',round(errorPosPost,2),isSuccessful,'VariableNames',{'N' 'Posicion E-E' 'Posicion hs' 'Error Pre-Placement' 'Posicion Aguja' 'Posicion Objetivo' 'Error Post-Placement' 'Exito'});
        %disp(t3);
        t4 = table(numTest',zEE_s',ns_s',round(errorOrientPre,2),zNeedle_s',ns_s',round(errorOrientPost,2),isSuccessful,'VariableNames',{'N' 'Orientación E-E' 'Orientación hs' 'Error Pre-Placement' 'Orientación de Aguja' 'Orientación de Inserción' 'Error Post-Placement' 'Exito'});
        %disp(t4);
        
        t5 = table(numTest',round(pT,2),round(hs,2)',round(errorPosPre,2),round(pNeedle,2),round(ps,2)',round(errorPosPost,2),round(zEE,2),round(ns,2)',round(errorOrientPre,2),round(zNeedle,2),round(ns,2)',round(errorOrientPost,2),isSuccessful,'VariableNames',{'num' 'Ts' 'Hs' 'ePrePos' 'pNeedle' 'Ps' 'ePostPos' 'zEE' 'ns' 'ePreOrient' 'zNeedle' 'Ns' 'ePostOrient' 'Success'});
        %disp(t5);
       
        %% Saving to excel-> Reporting data
        if needleType == 1
            nameExcel = mainPath+"\Simulink_models\Results\simulationForth"+cupSize+"_FNA"+".xls";
        else
            nameExcel = mainPath+"\Simulink_models\Results\simulationForth"+cupSize+"_CN"+".xls";
        end
        
        %writetable(t1,nameExcel,'Sheet',1,'WriteRowNames',true);%Uncomment if needed
        writetable(t2,nameExcel,'Sheet',2);
        writetable(t3,nameExcel,'Sheet',3);
        writetable(t4,nameExcel,'Sheet',4);
        writetable(t5,nameExcel,'Sheet',5);
    end
end

%% Plotting output data if needed
%Using simulation data manager 
%Simulink.sdi.clear
%Simulink.sdi.openVariable('simOutputs', simOutputs)
