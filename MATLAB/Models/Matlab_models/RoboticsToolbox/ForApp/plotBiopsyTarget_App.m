function [MidSegment,alpha90,nBoundary,mechUpperDelta,mechLowerDelta,collision1,collision2,collision3,numLidToOpen,doubleCase,inevitableCollision] = ...
    plotBiopsyTarget_App(ax,p_c,cupSize,u,v,nGridOpening,robotData,holderData,holderModel,rotationAngle)
%% Plotting code for our free-collision needle path algorithm
% INPUTS
% ax: Axes for plotting in an Matlab App
% p_c: Biopsy target position in frame {c}
% cupSize: Breast cup size for breast holder devices (A, B, C and D)
% u,w: Auxiliar vectors
% nGridOpening: Number of openings in the holder device for each removed
% lid. By default is 3.
% robotData: 
% holderData: Measurements of each Breast Holder Device
% holderModel:
% rotationAngle: Angle for vertical rotation of the breast holder device to avoid type 2 collisions
% OUTPUTS
% MidSegment: Segment position in which the needle intersects the holder
% device
% alpha90: Angle of needle insertion using the shortest distance approach
% nBoundary: Vector positions for maximum variation of insertion angle
% mechUpperDelta: Upper (mechanical) angle for needle insertion
% realUpperDelta: Final upper angle for needle insertion
% mechLowerDelta: Lower (mechanical) angle for needle insertion
% realLowerDelta: Final lower angle for needle insertion
% collision1, collision2, collision3: Type 1, 2 and 3 collisions 
% numLidToOpen: Order tag of lid to be opened
% doubleCase: Variable to identify if there are one or two openings for
% insertion
% inevitableCollision: If there's possible any needle insertion
%
% PARTS OF A HOLDER DEVICE
% - Holder structure
% - Columns : Vertical supports
% - Rows : Horizontal supports
% - Lids
% --------------------------------------------------------------
clc
%% Step 0: Loading robot and holder data
R_upper = holderData.R_upper;
R_lower = holderData.R_lower;
H_holder = holderData.H_holder;
phi = holderData.phi;
T_holder = holderData.T_holder;
H_table = holderData.H_table;
q_2max = robotData.q_2max;
L0 = robotData.L0;
H1 = robotData.H1;
L1 = robotData.L1;
L2 = robotData.L2;
H2 = robotData.H2;

%% Step 1: Conditions for the shortest distance. u and v are perpendicular
beta = pi/2;
alpha90 = beta - phi;%opening angle between u and n90

%% Step 2: A direct method to find the unit vector n by rotating u
screwMatrix= VecToso3(-v);
Rb90 = MatrixExp3s(screwMatrix,alpha90);
n90 = Rb90*u;

%% Step 3 : Unit vector w along the holder generatrix' side on a radial plane
pIntUpperLimit = [0 0 0]' + u*R_upper;% Upper point at inner surface cone (minor radius base)
pIntLowerLimit = [0 0 H_holder]' + u*R_lower;% Lower point at inner surface cone (major radius)
pOutUpperLimit = pIntUpperLimit + T_holder*n90;% Upper point at outer surface cone (holder minor radius)
pOutLowerLimit = pIntLowerLimit + T_holder*n90;% Lower point at outer surface cone (holder major radius)
w = (pIntLowerLimit-pIntUpperLimit)/norm(pIntLowerLimit-pIntUpperLimit);%

%% Step 4: Finding shortest distance using a vector formulation
pTob90 = (pIntUpperLimit-p_c)-((pIntUpperLimit-p_c)'*w)*w;
dShortest = norm(pTob90);
a90 = p_c + n90*(dShortest+T_holder);

%% Step 5 : Mechanical limits por alpha between a q_2min and q_2max
dTableToQ2min = H_table - (L0 + L1 + L2 + H2);
pQ2min = H1*u + dTableToQ2min*[0 0 1]';
pQ2max = pQ2min + q_2max*[0 0 -1]';
m = (pQ2max-p_c)/norm(pQ2max-p_c);%Unit vector for a maximum upper insertion 
n = (pQ2min-p_c)/norm(pQ2min-p_c);%Unit vector for a minimum lower insertion

%% Step 6 : Plotting the breast holder profile
% The holder profile was taken as the base of this algorithm
[gridPointInt,gridPointOut,nGridRow] = plotHolderProfile_App(ax,n90,u,w,nGridOpening,cupSize,holderData);

% Plotting the needle path under the shortest distance approach
pathNeedle90 = [p_c';a90'];
line(ax,pathNeedle90(:,1),pathNeedle90(:,2),pathNeedle90(:,3),'Color','blue','LineStyle',':','LineWidth',1);

plot3(ax,0,0,0,'ko','MarkerSize',10,'MarkerFaceColor','k');
plot3(ax,p_c(1),p_c(2),p_c(3),'ro','MarkerSize',8,'MarkerFaceColor','r');
plot3(ax,a90(1),a90(2),a90(3),'bo','MarkerSize',8,'MarkerFaceColor','b');

% Adding text to some reference points
aText = a90 + 4*u;
pcText = p_c - 1*u;
xText = [0 pcText(1) aText(1)];
yText = [0 pcText(2) aText(2)];
zText = [0 pcText(3) aText(3)];
Text = {'\leftarrow \{c\}','\leftarrow p_c','a \rightarrow'};
text(ax,xText,yText,zText,Text,'Color','red','FontSize',14);

%% Step 7: Computing the end effector's mechanical limits
% For the upper outer edge
pcToOutUpperLimit = pOutUpperLimit-p_c;
uPcToOutUpperLimit = pcToOutUpperLimit/norm(pcToOutUpperLimit);
theta1 = acos(m'*uPcToOutUpperLimit);
theta2 = acos(m'*w);
% For the lower outer edge
pcToOutLowerLimit = pOutLowerLimit-p_c;
uPcToOutLowerLimit = pcToOutLowerLimit/norm(pcToOutLowerLimit);
theta3 = acos(n'*uPcToOutLowerLimit);
theta4 = acos(n'*-w);

d_pcToQ2max = norm(pcToOutUpperLimit)*sin(pi-theta1-theta2)/sin(theta2);
d_pcToQ2min = norm(pcToOutLowerLimit)*sin(pi-theta3-theta4)/sin(theta4);

% Vector positions along the cone generatrix
pUppMechLim = p_c + d_pcToQ2max*m;
pLowMechLim = p_c + d_pcToQ2min*n;
mechLimit1 = [p_c';pUppMechLim'];
mechLimit2 = [p_c';pLowMechLim'];

% Plotting insertion directions due to mechanical limits of end effector movement 
line(ax,mechLimit1(:,1),mechLimit1(:,2),mechLimit1(:,3),'Color','black','LineStyle','-.','LineWidth',1);
line(ax,mechLimit2(:,1),mechLimit2(:,2),mechLimit2(:,3),'Color','black','LineStyle','-.','LineWidth',1);

%% Step 8: Finding holder's interval of insertion
% Making a search of the right interval in all partitions for target insertion
% The divisions for partitions are measured from one corner to another along cone generatrix.
% All distances are always measured from a fixed point in outer surface of holder (pOutUpperLimit)
d_ToMidSegment = norm(a90 - pOutUpperLimit);%point to locate
d_ToUppMechLim = norm(pUppMechLim-pOutUpperLimit);
d_ToLowMechLim = norm(pLowMechLim-pOutUpperLimit);
d_ToSegment = zeros(nGridRow,1);

for i=1:nGridRow
    d_ToSegment(i) = norm(gridPointOut(i,:) - pOutUpperLimit');
end

% Finding the interval in wich point a90 intersects
UpperSegment = 0;% Upper interval for a maximum mechanical elevation of joint 2 (q_2max)
LowerSegment = 0;% Lower interval for a minimum mechanical elevation of joint 2 (q_2min)
errorTarget  = false;

if d_ToMidSegment>0 && d_ToMidSegment<d_ToSegment(end)
    MidSegment = find(d_ToMidSegment<d_ToSegment,1)-1;
    UpperSegment = find(d_ToUppMechLim<d_ToSegment,1)-1;
    LowerSegment = find(d_ToLowMechLim<d_ToSegment,1)-1;
else
    % Some Restrictions
    if d_ToMidSegment>d_ToSegment(end)
        MidSegment = 7;
    elseif d_ToMidSegment<0
        MidSegment = 1;
    else
        errorTarget = true;
    end
end

%% Step 9: Computing limit angles for insertion
boundaryPoint = zeros(4,3);
nBoundary = boundaryPoint;
realUpperDelta = acos(n90'*m);
realLowerDelta = acos(n90'*n);

angleError = 1*pi/180;% For avoiding colision during insertion
inevitableCollision = 0;% For unavoidable collision. No insertion opening
mechUpperDelta = 0;% Mechanical upper insertion direction
mechLowerDelta = 0;% Mechanical lower insertion direction

% ::NOTE::
% doubleCase : True --> There are two openings for insertion
% doubleCase: False --> There's just one opening for insertion
doubleCase = false;

%% Step 10: Verifying vertical and horizontal collisions with breast holder device
needleType = 2;%A first insertion approach was tested with a FNA needle
[collision1,collision2,collision3,numLidToOpen]= isCollision_App(p_c,MidSegment,dShortest+T_holder,n90,v,cupSize,needleType,R_upper,holderModel,rotationAngle);

%% Step 11 : Finding a right needle insertion interval and other neighbour intervals
% Note: If there's collision  2 --> It migh be fixed by rotating the holder device

% Estrategy: No collisions with breast holder columns (Type 2 collision)
if ~collision2 && ~errorTarget

    % *** General Case 1: No Collision type 1 ***
    if ~collision1 && (MidSegment~=1 || MidSegment~=nGridRow-1)
        % To verify if there are collisions of limits 1 or 4 with next neighbour rows
        auxB1 = false;
        auxB4 = auxB1;

        % Limits for needle insertion. Possible collisions with neighbour segments from the target point
        nInitialBoundary(1,:) = (gridPointInt(MidSegment-1,:)'-p_c)/norm(gridPointInt(MidSegment-1,:)'-p_c);
        nInitialBoundary(2,:) = (gridPointOut(MidSegment,:)'-p_c)/norm(gridPointOut(MidSegment,:)'-p_c);
        nInitialBoundary(3,:) = (gridPointOut(MidSegment+1,:)'-p_c)/norm(gridPointOut(MidSegment+1,:)'-p_c);
        nInitialBoundary(4,:) = (gridPointInt(MidSegment+2,:)'-p_c)/norm(gridPointInt(MidSegment+2,:)'-p_c);
        limitAngle(1) = acos(nInitialBoundary(1,:)*n90);
        limitAngle(2) = acos(nInitialBoundary(2,:)*n90);
        limitAngle(3) = acos(nInitialBoundary(3,:)*n90);
        limitAngle(4) = acos(nInitialBoundary(4,:)*n90);

        % Verifying if the mechanical limits are in collision with the closest segments (holder rows)
        if realUpperDelta<(limitAngle(1)+angleError) && realUpperDelta>limitAngle(2)
            auxB1=true;
        end
        if realLowerDelta<(limitAngle(4)+angleError) && realLowerDelta>limitAngle(3)
            auxB4 = true;
        end

        % At least one of the robot limits is outside the closest segments
        if realUpperDelta>limitAngle(1) || realLowerDelta>limitAngle(4)
            doubleCase = true;
        end
        
        % Default vector configuration
        boundaryPoint(1,:) = pUppMechLim;
        boundaryPoint(4,:) = pLowMechLim;
        boundaryPoint(2,:) = [0 0 0]';%There's not limit
        boundaryPoint(3,:) = [0 0 0]';%There's not limit
        nBoundary(1,:) = m;
        nBoundary(2,:) = n90;
        nBoundary(3,:) = n90;
        nBoundary(4,:) = n;
        
        % If both mechanical limits (maximum and minimum insertions) are in different intervals
        % If mechanical limits are in collision with row supports then choosing a safer direction
        if UpperSegment~=LowerSegment
            % First: is there a collision with the maximum upper limit?
            if auxB1 % If so, limiting the insertion path inside of safer boundary
                boundaryPoint(1,:) = gridPointOut(MidSegment,:)';
                nBoundary(1,:) = (boundaryPoint(1,:)'-p_c)/norm(boundaryPoint(1,:)'-p_c);
                disp('A superior collision with breast holder. Limited')
            else
                % If not, continuing identifying limits
                if MidSegment==UpperSegment
                    if doubleCase == true
                        if LowerSegment~=0
                            % It's not a superior limit situation
                        else
                            % It's a superior limit situation
                            disp('The upper segment is outside the holder profile')
                        end

                        % Defining an auxiliar vector 
                        auxDirection = (gridPointInt(MidSegment+1,:)'-p_c) - ((gridPointInt(MidSegment+1,:)'-p_c)'*n90)*n90;
                        if auxDirection(3)>0% Along Zc direcion. Coordinate system of breast holder
                            boundaryPoint(2,:) = gridPointOut(MidSegment+1,:)';
                        else
                            boundaryPoint(2,:) = gridPointInt(MidSegment+1,:)';
                        end
                        nBoundary(2,:) = (boundaryPoint(2,:)'-p_c)/norm(boundaryPoint(2,:)'-p_c);
                    else
                        % No double case. Boundary point 2 is defined by default configuration
                    end                   
                else % If not, the upper limit is in an possible superior segment
                    doubleCase = true;
                    auxDirection = (gridPointInt(MidSegment-1,:)'-p_c) - ((gridPointInt(MidSegment-1,:)'-p_c)'*n90)*n90;
                    if auxDirection(3)>0
                        boundaryPoint(2,:) = gridPointOut(MidSegment-1,:)';
                    else
                        boundaryPoint(2,:) = gridPointInt(MidSegment-1,:)';
                    end
                    nBoundary(2,:) = (boundaryPoint(2,:)'-p_c)/norm(boundaryPoint(2,:)'-p_c);
                end
                
            end
            
            % Second: is there collision with the lower limit?
            if auxB4
                boundaryPoint(4,:) = gridPointOut(MidSegment+1,:)';
                nBoundary(4,:) = (boundaryPoint(4,:)'-p_c)/norm(boundaryPoint(4,:)'-p_c);
                disp('An inferior collision with breast holder. Limited')
            else
                % The lower limit is outside the neighbour holder raw
                if MidSegment==LowerSegment
                    if doubleCase==true
                        if UpperSegment~=0
                            % It's not a superior limit situation
                        else
                            % It's a superior limit situation
                            disp('The lower segment is outside the holder profile')
                        end%*
                        auxDirection = (gridPointInt(MidSegment,:)'-p_c) - ((gridPointInt(MidSegment,:)'-p_c)'*n90)*n90;
                        if auxDirection(3)>0
                            boundaryPoint(3,:) = gridPointInt(MidSegment,:)';
                        else
                            boundaryPoint(3,:) = gridPointOut(MidSegment,:)';
                        end
                        nBoundary(3,:) = (boundaryPoint(3,:)'-p_c)/norm(boundaryPoint(3,:)'-p_c);
                    else
                        % No double case. Boundary point 2 by default configuration
                    end     
                else % If not, the lower limit is in an possible lower segment
                    doubleCase = true;
                    auxDirection = (gridPointInt(MidSegment+2,:)'-p_c) - ((gridPointInt(MidSegment+2,:)'-p_c)'*n90)*n90;
                    if auxDirection(3)>0 % Defined in {C} frame. Zc points to holder lower radius
                        boundaryPoint(3,:) = gridPointInt(MidSegment+2,:)';
                    else
                        % Not posible position, but considered anyway.
                        boundaryPoint(3,:) = gridPointOut(MidSegment+2,:)';
                    end
                    nBoundary(3,:) = (boundaryPoint(3,:)'-p_c)/norm(boundaryPoint(3,:)'-p_c);
                end
            end
                         
        else
            % Both mechanical limits are in the same segment
            % Defining the right segment like the closest one.
            % The limits are by default
            % doubleCase = false;
        end
    
    % *** General Case 2: Type 1 collision ***
    else
        if MidSegment==1
            boundaryPoint(1,:) = [0 0 0]';
            boundaryPoint(2,:) = [0 0 0]';
            nBoundary(1,:) = [0 0 0]';
            nBoundary(2,:) = [0 0 0]';
            
            % Colision with the closest row
            nInitialBoundary(3,:) = (gridPointInt(MidSegment+1,:)'-p_c)/norm(gridPointInt(MidSegment+1,:)'-p_c);
            limitAngle(3) = acos(nInitialBoundary(3,:)*n90);
            if realLowerDelta>limitAngle(3)
                auxDirection = (gridPointInt(MidSegment+1,:)'-p_c) - ((gridPointInt(MidSegment+1,:)'-p_c)'*n90)*n90;
                if auxDirection(3)>0
                    boundaryPoint(3,:) = gridPointInt(MidSegment+1,:);
                else
                    boundaryPoint(3,:) = gridPointOut(MidSegment+1,:);
                end
                nBoundary(3,:) = (boundaryPoint(3,:)'-p_c)/norm(boundaryPoint(3,:)'-p_c);
            else
                % Upper limit is already in colision by default
                inevitableCollision = 1;%Case 5: target is no reachable
                boundaryPoint(3,:) = [0 0 0];
                nBoundary(3,:) = [0 0 0];
                disp("Error: Lower limit always in colision ")
            end

            % Colision with a lower row
            nInitialBoundary(4,:) = (gridPointOut(MidSegment+2,:)'-p_c)/norm(gridPointOut(MidSegment+2,:)'-p_c);
            limitAngle(4) = acos(nInitialBoundary(4,:)*n90);
            if realLowerDelta<limitAngle(4)
                boundaryPoint(4,:) = pLowMechLim;
                nBoundary(4,:) = n;
            else
                boundaryPoint(4,:) = gridPointOut(MidSegment+2,:);
                nBoundary(4,:) = (boundaryPoint(4,:)'-p_c)/norm(boundaryPoint(4,:)'-p_c);
            end         
                
        elseif MidSegment==7
            % Colision with an upper row
            nInitialBoundary(1,:) = (gridPointOut(MidSegment-1,:)'-p_c)/norm(gridPointOut(MidSegment-1,:)'-p_c);
            limitAngle(1) = acos(nInitialBoundary(1,:)*n90);
            if realUpperDelta<limitAngle(1)
                boundaryPoint(1,:) = pUppMechLim;
                nBoundary(1,:) = m;
            else
                boundaryPoint(1,:) = gridPointOut(MidSegment-1,:);
                nBoundary(1,:) = (boundaryPoint(1,:)'-p_c)/norm(boundaryPoint(1,:)'-p_c);
            end  
            
            % Colision with the closest row
            nInitialBoundary(2,:) = (gridPointInt(MidSegment,:)'-p_c)/norm(gridPointInt(MidSegment,:)'-p_c);
            limitAngle(2) = acos(nInitialBoundary(2,:)*n90);
            if realUpperDelta>limitAngle(2)
                auxDirection = (gridPointInt(MidSegment,:)'-p_c) - ((gridPointInt(MidSegment,:)'-p_c)'*n90)*n90;
                if auxDirection(3)>0
                    boundaryPoint(2,:) = gridPointOut(MidSegment,:);
                else
                    boundaryPoint(2,:) = gridPointInt(MidSegment,:);
                end
                nBoundary(2,:) = (boundaryPoint(2,:)'-p_c)/norm(boundaryPoint(2,:)'-p_c);
              
            else
                % Lower limit is already in colision by default
                inevitableCollision = 1;%Case 5: target is no reachable
                boundaryPoint(2,:) = [0 0 0];
                nBoundary(2,:) = [0 0 0];
                disp("Error: Upper limit always in colision")
            end
            
            boundaryPoint(3,:) = [0 0 0]';
            boundaryPoint(4,:) = [0 0 0]';
            nBoundary(3,:) = [0 0 0]';
            nBoundary(4,:) = [0 0 0]';
        else
            % To Verify if collision with the closest holder row is avoidable
            aux1 = false;
            aux2 = aux1;
            % To verify collision of limits 1 and 4 with the next neighbour rows
            auxB1 = false;
            auxB4 = auxB1;
            
            % Boundaries to nearest collision by some holder rows
            nInitialBoundary(2,:) = (gridPointInt(MidSegment,:)'-p_c)/norm(gridPointInt(MidSegment,:)'-p_c);
            nInitialBoundary(3,:) = (gridPointInt(MidSegment+1,:)'-p_c)/norm(gridPointInt(MidSegment+1,:)'-p_c);
            limitAngle(2) = acos(nInitialBoundary(2,:)*n90);      
            limitAngle(3) = acos(nInitialBoundary(3,:)*n90);
            % Default initial limits
            nInitialBoundary(1,:) = (gridPointOut(MidSegment-1,:)'-p_c)/norm(gridPointOut(MidSegment-1,:)'-p_c);
            nInitialBoundary(4,:) = (gridPointOut(MidSegment+2,:)'-p_c)/norm(gridPointOut(MidSegment+2,:)'-p_c);
            limitAngle(1) = acos(nInitialBoundary(1,:)*n90);
            limitAngle(4) = acos(nInitialBoundary(4,:)*n90);
            
            % Verifying if mechanical limits are in posible collision with
            % neighbour segments. A simple approach:
            % Assuming that a grater realUpperDelta than limitAngle can cause a collision
            if realUpperDelta>limitAngle(1)
                auxB1 = true;
            end
            if realLowerDelta>limitAngle(4)
                auxB4 = true;
            end
        
            % Verify collision with closest neighbour segments         
            if realUpperDelta>limitAngle(2)
                % So far, doubleCase could be true
                if auxB1==true%The limit 1 is in collision
                    boundaryPoint(1,:) = gridPointOut(MidSegment-1,:);
                    nBoundary(1,:) = (boundaryPoint(1,:)'-p_c)/norm(boundaryPoint(1,:)'-p_c);
                else% By default
                    boundaryPoint(1,:) = pUppMechLim;
                    nBoundary(1,:) = m;
                end
                
                auxDirection1 = (gridPointInt(MidSegment,:)'-p_c) - ((gridPointInt(MidSegment,:)'-p_c)'*n90)*n90;
                if auxDirection1(3)>0
                    boundaryPoint(2,:) = gridPointOut(MidSegment,:);
                else
                    boundaryPoint(2,:) = gridPointInt(MidSegment,:);
                end
                nBoundary(2,:) = (boundaryPoint(2,:)'-p_c)/norm(boundaryPoint(2,:)'-p_c);
            else
                disp("Error: Upper limit is always in collision")
                aux1 = true;
                boundaryPoint(2,:) = [0 0 0];
                nBoundary(2,:) = [0 0 0];
            end
            
            if realLowerDelta>limitAngle(3)
                % So far, doubleCase could be true;
                auxDirection2 = (gridPointInt(MidSegment+1,:)'-p_c) - ((gridPointInt(MidSegment+1,:)'-p_c)'*n90)*n90;
                if auxDirection2(3)>0
                    boundaryPoint(3,:) = gridPointInt(MidSegment+1,:);
                else
                    boundaryPoint(3,:) = gridPointOut(MidSegment+1,:);
                end
                nBoundary(3,:) = (boundaryPoint(3,:)'-p_c)/norm(boundaryPoint(3,:)'-p_c);
                
                if auxB4==true%The limit 4 is in collision
                    boundaryPoint(4,:) = gridPointOut(MidSegment+2,:);
                    nBoundary(4,:) = (boundaryPoint(4,:)'-p_c)/norm(boundaryPoint(4,:)'-p_c);
                else% By default
                    boundaryPoint(4,:) = pLowMechLim;
                    nBoundary(4,:) = n;
                end
            else
                disp("Error: Lower limit is always in collision")
                aux2=true;
                boundaryPoint(3,:) = [0 0 0];
                nBoundary(3,:) = [0 0 0];
            end
            
            % Identifying a possible Case 5
            if aux1 && aux2
                inevitableCollision = 1;%Case 5: target is no reachable
            elseif aux1 && ~aux2
                inevitableCollision = 2;%Case 5: upper opening in collision
            elseif ~aux1 && aux2
                inevitableCollision = 3;%Case 5: lower opening in collision
            elseif ~aux1 && ~aux2
                doubleCase = true;% No Case 5: double case is right
            end
                
        end
    end

    % Possible angle ranges to choose
    mechUpperDelta = acos(nBoundary(1,:)*nBoundary(2,:)');
    mechLowerDelta = acos(nBoundary(3,:)*nBoundary(4,:)');

    % Correcting angles for no possible collisions
    if mechUpperDelta == pi/2
        mechUpperDelta = 0;
    end
    if mechLowerDelta == pi/2
        mechLowerDelta = 0;
    end
    
    colores = {'red',[0 0.8 0],[0.5 0 1],'blue'};

    % Plotting mechanical limits for needle insertion
    for i=1:4
        if sum(boundaryPoint(i,:)) ~= 0
            arregloPlot = [p_c';boundaryPoint(i,:)];
            plot3(ax,arregloPlot(:,1),arregloPlot(:,2),arregloPlot(:,3),'Color',colores{i},'LineStyle','--','LineWidth',2);
        end
    end
end
view(ax,v);

disp("Real limit angles")
disp(realUpperDelta)
disp(realLowerDelta)
disp("Mechanical limit angles")
disp(mechUpperDelta)
disp(mechLowerDelta)
fprintf("Double: %d y error: %d\n",doubleCase,inevitableCollision)
end