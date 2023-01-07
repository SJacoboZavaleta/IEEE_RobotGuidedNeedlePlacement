function [MidSegment,alpha90,mechUpperDelta,realUpperDelta,mechLowerDelta,realLowerDelta,collision1,collision2,collision3,numLeadToOpen,doubleCase,nBoundary,inevitableCollision] = ...
    plotBiopsyTarget_Sim(pc,cupSize,needleType,needleGauge,u,v,nGridOpening,robotData,holderData,holderModel,rotationAngle)
%% Plotting code for our free-collision needle path algorithm
% INPUTS
% ax: Axes for plotting in an Matlab App
% p_c: Vector position for biopsy target in frame {C}
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
% mechUpperDelta: Upper (mechanical) angle for needle insertion
% realUpperDelta: Final upper angle for needle insertion
% mechLowerDelta: Lower (mechanical) angle for needle insertion
% realLowerDelta: Final lower angle for needle insertion
% collision1, collision2, collision3: Type 1, 2 and 3 collisions
% numLeadToOpen: Order tag of lid to be opened
% doubleCase: Variable to identify if there are one or two openings for
% insertion
% nBoundary: Vector positions for needle insertion limits 
% inevitableCollision: If there's possible any needle insertion
%
% PARTS OF A HOLDER DEVICE
% - Holder structure
% - Columns : Vertical supports
% - Rows : Horizontal supports
% - Lids
% --------------------------------------------------------------
%% Step 0: Loading needed data
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

%% Step 2: A direct method to find unit vector n by rotating u
screwMatrix= VecToso3(-v);
Rb90 = MatrixExp3s(screwMatrix,alpha90);
n90 = Rb90*u;

%% Step 3 : Unit vector w along the generatrix holderData side on radial plane
pIntUpperLimit = [0 0 0]' + u*R_upper;%Upper Internal cone border
pIntLowerLimit = [0 0 H_holder]' + u*R_lower;%Lower Internal coner border
pOutUpperLimit = pIntUpperLimit + T_holder*n90;%External border point at superior limit of cone base
pOutLowerLimit = pIntLowerLimit + T_holder*n90;%outer surface point at inferior limit of cone base
w = (pIntLowerLimit-pIntUpperLimit)/norm(pIntLowerLimit-pIntUpperLimit);%

%% Step 4: Finding shortest distance using a vector formulation
pTob90 = (pIntUpperLimit-pc)-((pIntUpperLimit-pc)'*w)*w;
dShortest = norm(pTob90);
a90 = pc + n90*(dShortest+T_holder);

%% Step 5 : Mechanical limits por alpha between a q_2min and q_2max
dTableToQ2min = H_table - (L0 + L1 + L2 + H2);
pQ2min = H1*u + dTableToQ2min*[0 0 1]';
pQ2max = pQ2min + q_2max*[0 0 -1]';
m = (pQ2max-pc)/norm(pQ2max-pc);%upperLimit unit vector
n = (pQ2min-pc)/norm(pQ2min-pc);%lowerLimit unit vector

%% Step 6 : Getting breast holder profile
[gridPointInt,gridPointOut,nGridRow] = plotHolderProfile_Sim(n90,u,w,nGridOpening,holderData);

%% Step 7: Computing Breast Holder's mechanical limits
% For upper outer edge
pcToOutUpperLimit = pOutUpperLimit-pc;
uPcToOutUpperLimit = pcToOutUpperLimit/norm(pcToOutUpperLimit);
theta1 = acos(m'*uPcToOutUpperLimit);
theta2 = acos(m'*w);
% For lower outer edge
pcToOutLowerLimit = pOutLowerLimit-pc;
uPcToOutLowerLimit = pcToOutLowerLimit/norm(pcToOutLowerLimit);
theta3 = acos(n'*uPcToOutLowerLimit);
theta4 = acos(n'*-w);

d_pcToQ2max = norm(pcToOutUpperLimit)*sin(pi-theta1-theta2)/sin(theta2);
d_pcToQ2min = norm(pcToOutLowerLimit)*sin(pi-theta3-theta4)/sin(theta4);

% Points in the cone generatrix
pUppMechLim = pc + d_pcToQ2max*m;
pLowMechLim = pc + d_pcToQ2min*n;

%% Step 8: Finding holder's interval of insertion
% Making a search interval in partitions for target insertion
% Measure from each point of holderData divisions to the upper border pOutUpperLimit along cone
% generatrix.
d_ToMidSegment = norm(a90 - pOutUpperLimit);%point to locate
d_ToUppMechLim = norm(pUppMechLim-pOutUpperLimit);
d_ToLowMechLim = norm(pLowMechLim-pOutUpperLimit);
d_ToSegment = zeros(nGridRow,1);

%Distances from pOutUpperLimit : a fixed point
for i=1:nGridRow
    d_ToSegment(i) = norm(gridPointOut(i,:) - pOutUpperLimit');
end

% Finding the interval or neighbourd of point a90
%MidSegment = 0;% main interval. The nearest one.
UpperSegment = 0;%upper limit for q_2max interval
LowerSegment = 0;%lower limit for q_2min interval
errorTarget  = false;
try
    if d_ToMidSegment>=0 && d_ToMidSegment<=d_ToSegment(end)
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
            MidSegment = 0;
            errorTarget = true;
        end
    end
catch ME
    error("a90 is wrong defined. Check other variables like u if it is NAN");
end

%% Step 9: Computing limit angles for insertion
boundaryPoint = zeros(4,3);
nBoundary = boundaryPoint;
realUpperDelta = acos(n90'*m);
realLowerDelta = acos(n90'*n);

angleError = 1*pi/180;%for avoiding colision during insertion
inevitableCollision = 0;% for unavoidable collision. No insertion opening
mechUpperDelta = 0;%mechanical upper insertion direction
mechLowerDelta = 0;%mechanical lower insertion direction
% doubleCase : True --> There's two openings for insertion
% doubleCase: False --> There's just one opening for insertion
doubleCase = false;

%% Step 10: Verifying vertical, horizontal collision with breast holderData device
collision1 = false;
collision2 = collision1;
collision3 = collision1;
numLeadToOpen = 0;
if ~errorTarget
    [collision1,collision2,collision3,numLeadToOpen]= ...
         isCollision_Sim(pc,MidSegment,dShortest+T_holder,n90,v,cupSize,needleType,needleGauge,R_upper,holderModel,rotationAngle);
end

%% Step 11 : Finding a right needle insertion interval and other neighbour intervals
% Note: If there's collision  2 --> It migh be fixed by rotating the holder device

% Estrategy: No collisions with breast holder columns (Type 2 collision)
if ~errorTarget %~collision2 && ~errorTarget
    % General Case 1: type 1 no Collision
    if ~collision1 && (MidSegment~=1 || MidSegment~=nGridRow-1)
        % To verify collision of limits 1 and 4 with next neighbour rows
        auxB1 = false;
        auxB4 = auxB1;
        % Limits with near neighbour segments to target point
        nInitialBoundary(1,:) = (gridPointInt(MidSegment-1,:)'-pc)/norm(gridPointInt(MidSegment-1,:)'-pc);
        nInitialBoundary(2,:) = (gridPointOut(MidSegment,:)'-pc)/norm(gridPointOut(MidSegment,:)'-pc);
        nInitialBoundary(3,:) = (gridPointOut(MidSegment+1,:)'-pc)/norm(gridPointOut(MidSegment+1,:)'-pc);
        nInitialBoundary(4,:) = (gridPointInt(MidSegment+2,:)'-pc)/norm(gridPointInt(MidSegment+2,:)'-pc);
        limitAngle(1) = acos(nInitialBoundary(1,:)*n90);
        limitAngle(2) = acos(nInitialBoundary(2,:)*n90);
        limitAngle(3) = acos(nInitialBoundary(3,:)*n90);
        limitAngle(4) = acos(nInitialBoundary(4,:)*n90);
        % Verify if the mechanical limits are in collision with the closest
        % segments (holder rows)
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
        % Default configuration
        boundaryPoint(1,:) = pUppMechLim;
        boundaryPoint(4,:) = pLowMechLim;
        boundaryPoint(2,:) = [0 0 0]';%there's not limit
        boundaryPoint(3,:) = [0 0 0]';%there's not limit
        nBoundary(1,:) = m;
        nBoundary(2,:) = n90;
        nBoundary(3,:) = n90;
        nBoundary(4,:) = n;
        
        % If both mechanical limits are in different intervals
        % They're not in collision with the closest neighbour segments
        % If mechanical limits are in collision then choosing a safer direction
        if UpperSegment~=LowerSegment
            
            % First: is there collision with the upper limit?
            if auxB1 % if so, Limiting to a safer boundary
                boundaryPoint(1,:) = gridPointOut(MidSegment,:)';
                nBoundary(1,:) = (boundaryPoint(1,:)'-pc)/norm(boundaryPoint(1,:)'-pc);
            else
                % if not, continuing identifying limits
                if MidSegment==UpperSegment
                    if doubleCase == true
                        if LowerSegment~=0
                            %It's not an inferior limit situation
                        else
                            %It's an inferior limit situation
                        end
                        auxDirection = (gridPointInt(MidSegment+1,:)'-pc) - ((gridPointInt(MidSegment+1,:)'-pc)'*n90)*n90;
                        if auxDirection(3)>0% in Zc direcion
                            boundaryPoint(2,:) = gridPointOut(MidSegment+1,:)';
                        else
                            boundaryPoint(2,:) = gridPointInt(MidSegment+1,:)';
                        end
                        nBoundary(2,:) = (boundaryPoint(2,:)'-pc)/norm(boundaryPoint(2,:)'-pc);
                    else
                        % No double case. Boundary 2 by default
                    end                   
                else % If not, the upper limit is in an possible superior segment
                    doubleCase = true;
                    auxDirection = (gridPointInt(MidSegment-1,:)'-pc) - ((gridPointInt(MidSegment-1,:)'-pc)'*n90)*n90;
                    if auxDirection(3)>0
                        boundaryPoint(2,:) = gridPointOut(MidSegment-1,:)';
                    else
                        boundaryPoint(2,:) = gridPointInt(MidSegment-1,:)';
                    end
                    nBoundary(2,:) = (boundaryPoint(2,:)'-pc)/norm(boundaryPoint(2,:)'-pc);
                end
                
            end
            
            % Second: is there collision with the lower limit?
            if auxB4
                boundaryPoint(4,:) = gridPointOut(MidSegment+1,:)';
                nBoundary(4,:) = (boundaryPoint(4,:)'-pc)/norm(boundaryPoint(4,:)'-pc);
            else
                % The lower limit is outside the neighbour holder raw
                if MidSegment==LowerSegment
                    if doubleCase==true
                        if UpperSegment~=0
                            %It's not a superior limit situation
                        else
                            %It's a superior limit situation
                        end%*
                        auxDirection = (gridPointInt(MidSegment,:)'-pc) - ((gridPointInt(MidSegment,:)'-pc)'*n90)*n90;
                        if auxDirection(3)>0
                            boundaryPoint(3,:) = gridPointInt(MidSegment,:)';
                        else
                            boundaryPoint(3,:) = gridPointOut(MidSegment,:)';
                        end
                        nBoundary(3,:) = (boundaryPoint(3,:)'-pc)/norm(boundaryPoint(3,:)'-pc);
                    else
                        % No double case. Boundary 3 by default
                    end     
                else % If not, the lower limit is in an possible lower segment
                    doubleCase = true;
                    auxDirection = (gridPointInt(MidSegment+2,:)'-pc) - ((gridPointInt(MidSegment+2,:)'-pc)'*n90)*n90;
                    if auxDirection(3)>0 % Defined in {C}. Zc points to holder lower radius
                        boundaryPoint(3,:) = gridPointInt(MidSegment+2,:)';
                    else
                        % Not posible position, but considered anyway.
                        boundaryPoint(3,:) = gridPointOut(MidSegment+2,:)';
                    end
                    nBoundary(3,:) = (boundaryPoint(3,:)'-pc)/norm(boundaryPoint(3,:)'-pc);
                end
            end
                         
        else
            %Both mechanical limits are in the same segment
            %Defining the right segment like the closest one.
            %The limits are by default
            %doubleCase = false;
        end
                
    % General Case 2: type 1 collision
    else
        if MidSegment==1
            boundaryPoint(1,:) = [0 0 0]';
            boundaryPoint(2,:) = [0 0 0]';
            nBoundary(1,:) = [0 0 0]';
            nBoundary(2,:) = [0 0 0]';
            
            % Colision with the closest row
            nInitialBoundary(3,:) = (gridPointInt(MidSegment+1,:)'-pc)/norm(gridPointInt(MidSegment+1,:)'-pc);
            limitAngle(3) = acos(nInitialBoundary(3,:)*n90);
            if realLowerDelta>limitAngle(3)
                auxDirection = (gridPointInt(MidSegment+1,:)'-pc) - ((gridPointInt(MidSegment+1,:)'-pc)'*n90)*n90;
                if auxDirection(3)>0
                    boundaryPoint(3,:) = gridPointInt(MidSegment+1,:);
                else
                    boundaryPoint(3,:) = gridPointOut(MidSegment+1,:);
                end
                nBoundary(3,:) = (boundaryPoint(3,:)'-pc)/norm(boundaryPoint(3,:)'-pc);
            else
                % Upper limit is already in colision by default
                inevitableCollision = 1;%Case 5: target is no reachable
                boundaryPoint(3,:) = [0 0 0];
                nBoundary(3,:) = [0 0 0];
            end
            % Colision with a lower row
            nInitialBoundary(4,:) = (gridPointOut(MidSegment+2,:)'-pc)/norm(gridPointOut(MidSegment+2,:)'-pc);
            limitAngle(4) = acos(nInitialBoundary(4,:)*n90);
            if realLowerDelta<limitAngle(4)
                boundaryPoint(4,:) = pLowMechLim;
                nBoundary(4,:) = n;
            else
                boundaryPoint(4,:) = gridPointOut(MidSegment+2,:);
                nBoundary(4,:) = (boundaryPoint(4,:)'-pc)/norm(boundaryPoint(4,:)'-pc);
            end         
                
        elseif MidSegment==7
            % Colision with a upper row
            nInitialBoundary(1,:) = (gridPointOut(MidSegment-1,:)'-pc)/norm(gridPointOut(MidSegment-1,:)'-pc);
            limitAngle(1) = acos(nInitialBoundary(1,:)*n90);
            if realUpperDelta<limitAngle(1)
                boundaryPoint(1,:) = pUppMechLim;
                nBoundary(1,:) = m;
            else
                boundaryPoint(1,:) = gridPointOut(MidSegment-1,:);
                nBoundary(1,:) = (boundaryPoint(1,:)'-pc)/norm(boundaryPoint(1,:)'-pc);
            end  
            
            % Colision with the closest row
            nInitialBoundary(2,:) = (gridPointInt(MidSegment,:)'-pc)/norm(gridPointInt(MidSegment,:)'-pc);
            limitAngle(2) = acos(nInitialBoundary(2,:)*n90);
            if realUpperDelta>limitAngle(2)
                auxDirection = (gridPointInt(MidSegment,:)'-pc) - ((gridPointInt(MidSegment,:)'-pc)'*n90)*n90;
                if auxDirection(3)>0
                    boundaryPoint(2,:) = gridPointOut(MidSegment,:);
                else
                    boundaryPoint(2,:) = gridPointInt(MidSegment,:);
                end
                nBoundary(2,:) = (boundaryPoint(2,:)'-pc)/norm(boundaryPoint(2,:)'-pc);
              
            else
                % Lower limit is already in colision by default
                inevitableCollision = 1;%Case 5: target is no reachable
                boundaryPoint(2,:) = [0 0 0];
                nBoundary(2,:) = [0 0 0];
            end
            
            boundaryPoint(3,:) = [0 0 0]';
            boundaryPoint(4,:) = [0 0 0]';
            nBoundary(3,:) = [0 0 0]';
            nBoundary(4,:) = [0 0 0]';
        else
            % To Verify if collision with the closest holder row is avoidable
            aux1 = false;
            aux2 = aux1;
            % To verify collision of limits 1 and 4 with next neighbour rows
            auxB1 = false;
            auxB4 = auxB1;
            
            % Boundaries to nearest collision by some holder rows
            nInitialBoundary(2,:) = (gridPointInt(MidSegment,:)'-pc)/norm(gridPointInt(MidSegment,:)'-pc);
            nInitialBoundary(3,:) = (gridPointInt(MidSegment+1,:)'-pc)/norm(gridPointInt(MidSegment+1,:)'-pc);
            limitAngle(2) = acos(nInitialBoundary(2,:)*n90);      
            limitAngle(3) = acos(nInitialBoundary(3,:)*n90);
            % Default initial limits
            nInitialBoundary(1,:) = (gridPointOut(MidSegment-1,:)'-pc)/norm(gridPointOut(MidSegment-1,:)'-pc);
            nInitialBoundary(4,:) = (gridPointOut(MidSegment+2,:)'-pc)/norm(gridPointOut(MidSegment+2,:)'-pc);
            limitAngle(1) = acos(nInitialBoundary(1,:)*n90);
            limitAngle(4) = acos(nInitialBoundary(4,:)*n90);
            
            % Verify if mechanical limits are in posible collision with
            % neighbour segments. A simple approach:
            % Assuming that a grater realUpperDelta than limitAngle can 
            % cause a collision
            if realUpperDelta>limitAngle(1)
                auxB1 = true;
            end
            if realLowerDelta>limitAngle(4)
                auxB4 = true;
            end
        
            % Verify collision with closest neighbour segments         
            if realUpperDelta>limitAngle(2)
                % so far, doubleCase could be true
                if auxB1==true%The limit 1 is in collision
                    boundaryPoint(1,:) = gridPointOut(MidSegment-1,:);
                    nBoundary(1,:) = (boundaryPoint(1,:)'-pc)/norm(boundaryPoint(1,:)'-pc);
                else% By default
                    boundaryPoint(1,:) = pUppMechLim;
                    nBoundary(1,:) = m;
                end
                
                auxDirection1 = (gridPointInt(MidSegment,:)'-pc) - ((gridPointInt(MidSegment,:)'-pc)'*n90)*n90;
                if auxDirection1(3)>0
                    boundaryPoint(2,:) = gridPointOut(MidSegment,:);
                else
                    boundaryPoint(2,:) = gridPointInt(MidSegment,:);
                end
                nBoundary(2,:) = (boundaryPoint(2,:)'-pc)/norm(boundaryPoint(2,:)'-pc);
            else
                aux1 = true;
                boundaryPoint(2,:) = [0 0 0];
                nBoundary(2,:) = [0 0 0];
            end
            
            if realLowerDelta>limitAngle(3)
                % so far, doubleCase could be true;
                auxDirection2 = (gridPointInt(MidSegment+1,:)'-pc) - ((gridPointInt(MidSegment+1,:)'-pc)'*n90)*n90;
                if auxDirection2(3)>0
                    boundaryPoint(3,:) = gridPointInt(MidSegment+1,:);
                else
                    boundaryPoint(3,:) = gridPointOut(MidSegment+1,:);
                end
                nBoundary(3,:) = (boundaryPoint(3,:)'-pc)/norm(boundaryPoint(3,:)'-pc);
                
                if auxB4==true%The limit 4 is in collision
                    boundaryPoint(4,:) = gridPointOut(MidSegment+2,:);
                    nBoundary(4,:) = (boundaryPoint(4,:)'-pc)/norm(boundaryPoint(4,:)'-pc);
                else% By default
                    boundaryPoint(4,:) = pLowMechLim;
                    nBoundary(4,:) = n;
                end
            else
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
end

end