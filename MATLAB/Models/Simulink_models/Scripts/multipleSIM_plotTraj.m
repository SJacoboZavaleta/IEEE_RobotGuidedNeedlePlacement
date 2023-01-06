%% Getting multiple random data
clear, clc
robotData = load('D:\nuevaTesis2020\DESARROLLO\MATLAB\ModeloNuevo\robotData.mat');
mdl = 'breastBiopsySIMtraj';
open_system(mdl,'loadonly');
listCupSize = {'A' 'B' 'C' 'D'};
listNeedleType = [1,2];
numTargets = 4;%at least 4

for x=1:4
    for y = 1:2
        cupSize = listCupSize{x};
        fileTarget = "D:/nuevaTesis2020/DESARROLLO/MATLAB/ModeloNuevo/biopsyData.mat";
        pointsData = load(fileTarget);
        if cupSize == 'A'
            biopsyPoint = pointsData.targetDataA;
        elseif cupSize == 'B'
            biopsyPoint = pointsData.targetDataB;
        elseif cupSize == 'C'
            biopsyPoint = pointsData.targetDataC;
        else
            biopsyPoint = pointsData.targetDataD;
        end
        
        numBiopsyPoints = length(biopsyPoint);
        
        s = RandStream('dsfmt19937','Seed',2021);%before 3
        pc = zeros(3,numTargets);
        for i=1:numTargets
            target = round(numBiopsyPoints*rand(s,1));
            pc(:,i) = biopsyPoint(target,:)';
        end
        
        % Automated Biopsy Target decision
        % getMultipleDataToSIM(p_c,angle,cupSize,needleType,offsetPrePlacement)
        % cupSize = 'A'
        % needleType = 1(FNA), 2(CN)
        % needleGauge = 11, 14, 18, 20, 21
        % NeedleThrow = 20
        % NeedleDeadSpace = 8
        %---------------------------------
        angle = 0;
        needleType = listNeedleType(y);
        offsetPre = 10;
        
        typeTrajInterp = 2;%cubic
        typeTrajMode = 1;
        if needleType==1
            needleGauge = 20;
        else
            needleGauge = 16;
        end
        %---------------------------------
        ps = zeros(3,numTargets);
        hs = zeros(3,numTargets);
        ns = zeros(3,numTargets);
        collision1 = zeros(1,numTargets);
        collision1Unavoidable = collision1;
        collision2 = collision1;
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
        for i=1:numTargets
            [ps(:,i),hs(:,i),ns(:,i),collision1(i),collision1Unavoidable(i),collision2(i),collision3(i),collision4(i),isJointLimit(i),isReachable(i),...
                numLeadToOpen(i),numLeadToCol(i),dInsertion(:,i),needleLength(i),rNeedle(i),needleThrow(i),needleDeadSpace(i),needleDeviceLength(i)] = ...
                getMultipleDataToSIM(pc(:,i),angle,cupSize,needleType,needleGauge,offsetPre);
        end
        
        %% Preparing simulation
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
        %% Simulation
        simOutputs = sim(simIn);
        %% Preparing data
        ps_n = round(ps,2);
        hs_n = round(hs,2);
        ns_n = round(ns,2);        
        %% Table 3, 4 y 5
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
        qd = zeros(5,1561,numTargets);
        qm = zeros(1561,5,numTargets);
        timeSeries = simOutputs(1,i).tout;
        for i=1:numTargets
            if isReachable(i)==1
                data = simOutputs(1,i).logsout;
                errorPosPre(i) = data.get('ePos').Values.Data(1000,:)*1000;
                errorPosPost(i) = data.get('ePos').Values.Data(end,:)*1000;
                errorOrientPre(i) = data.get('eOrient').Values.Data(1000,:)*1000;
                errorOrientPost(i) = data.get('eOrient').Values.Data(end,:)*1000;
                qd(:,:,i) = data.get('q_d').Values.Data(:,:);
                qm(:,:,i) = data.get('q_m').Values.Data(:,:);
                %------------------------------
                timeSeries = data.get('pNeedle').Values.Time;
                aux = find(timeSeries>20.5);
                %------------------------------
                errorDeviceLength(i) = data.get('errorDevice').Values.Data(end);
                isSuccessful(i) = data.get('biopsySuccess').Values.Data(end);
            end
        end
        
        %% Saving to excel
        for i=1:numTargets
            q1d = qd(1,:,i)';
            q2d = qd(2,:,i)';
            q3d = qd(3,:,i)';
            q4d = qd(4,:,i)';
            q5d = qd(5,:,i)';
            q1 = qm(:,1,i);
            q2 = qm(:,2,i);
            q3 = qm(:,3,i);
            q4 = qm(:,4,i);
            q5 = qm(:,5,i);
            t1 = table(ps_n',hs_n',ns_n',isSuccessful,'VariableNames',{'ps (mm)' 'hs (mm)' 'ns' 'Successful'});
            t2 = table(q1d,q1,q2d,q2,q3d,q3,q4d,q4,q5d,q5,'VariableNames',{'q1d' 'q1' 'q2d' 'q2' 'q3d' 'q3' 'q4d' 'q4' 'q5d' 'q5'});
            if i==1
                if needleType == 1
                    nameExcel = "plotTraj"+cupSize+"_FNA"+".xls";
                    writetable(t1,nameExcel,'Sheet',1,'WriteRowNames',true);
                    writetable(t2,nameExcel,'Sheet',2);
                elseif needleType == 2
                    nameExcel = "plotTraj"+cupSize+"_CN"+".xls";
                    writetable(t1,nameExcel,'Sheet',1,'WriteRowNames',true);
                    writetable(t2,nameExcel,'Sheet',2);
                end
            else
                writetable(t2,nameExcel,'Sheet',i+1);%3,4,..
            end
        end
    end
end
%Using simulation data manager 
%Simulink.sdi.clear
%Simulink.sdi.openVariable('simOutputs', simOutputs)
