function [p_s,h_s,n_s,collision1,iCollision1,pCollision2,collision2,wasRotated,collision3,isJointLimit,isReachable,numLeadToOpen,...
    needleDistInsertion,needleLength,rNeedle,needleThrow,needleDeadSpace,needleDeviceLength,...
    mechUpperDelta,realUpperDelta,mechLowerDelta,realLowerDelta] = ...
    getMultipleData_Sim(p_c,cupSize,needleType,needleGauge,offsetPrePlacement,robotData,holderData,holderModel)
%% Automated script to get needed data for multiple simulation at once in Simulink
% INPUTS
% p_c:Breast target position in frame {c}
% cupSize: Breast cup size (A, B, C and D)
% needleType: Needle type (FNA and CN)
% needleGauge: Needle gauge (14, 16, 20, 22, etc)
% offsetPrePlacement: Fixed distance between end effector and breast holder
%                     after preplacement
% robotData: Needed robot data
% holderData: Breast holder data
% holderModel: Breast holder models
% OUTPUTS
% p_s: Biopsy target position in {s} frame
% h_s: Final end effector position after preplacement in {s} frame
% n_s: Desired needle insertion in {s] frame
% collision1: type 1 collision (horizontal supports of breast holder)
% iCollision1: inevitable type 1 collision 
% pCollision2: first type 2 collision (before holder rotation)
% collision2: final type 2 collison (after holder rotation)
% wasRotated: if it was needed to rotate holder to avoid type 2 collision 
% collision3: type 3 collision (risk of neumotorax at chest wall)
% isJointLimit: If joint limits are exceded by kinematic limitations
% isReachable: If target can theoretically be reached by the needle
% numLeadToOpen: Lid to be opened for needle insertion 
% numLeadToCol: 
% needleDistInsertion: Distance from outer holder surface and biopsy target
% position
% needleLength: Needle length from base to its tip
% rNeedle: External needle radius
% needleThrow: Throw distance for the sampling notch in core needles
% needleDeadSpace: Needle tip length in core needles
% needleDeviceLength: Needle device length 
% mechUpperDelta: Upper (mechanical) angle for needle insertion
% realUpperDelta: Final upper angle  for needle insertion
% mechLowerDelta: Lower (mechanical) angle for needle insertion
% realLowerDelta: Final lower angle for needle insertion
% ------------------------------------
%% Step 0: Loading needed data
nGridOpening = 3;
R_holderUpper = holderData.R_upper;
R_holderLower = holderData.R_lower;
H_holder = holderData.H_holder;
phi = holderData.phi;
T_holder = holderData.T_holder;

%% Step 1: Finding basic unit vectors for XoYoZo Projection

if p_c(1)+p_c(2)==0 % Avoiding target positions like [0 0 10]
    % A new p_c is created
    p_c = p_c+ 1*[1 1 0]';% A 1 mm offset in x and y positions
end    

rxy = sqrt(p_c(1)^2 + p_c(2)^2);
u = [p_c(1) p_c(2) 0]' / rxy;% horizontal vector on radial plane
v = [-p_c(2) p_c(1) 0]'/rxy;% perpendicular vector to u and a radial plane

% On the radial plane. RadiusP is the horizonal distance between the vertical 
% breast holder axis (Zc) and the biopsy target position
radiusP = (R_holderUpper-R_holderLower)*(H_holder-p_c(3))/H_holder + R_holderLower;%Defined as r !
d1 = radiusP - rxy;

% Defining a point C as a horizontal projection of biopsy target on outer
% holder surface
c = p_c + d1*u;
c_ext = c + u*T_holder/sin(phi);

%% Step 2 : Finding breast holder profile and collisions
pCollision2 = false;
collision2 = pCollision2;
wasRotated = 0;
breastHolderRotationAngle = 0;
[MidSegment,alpha90,mechUpperDelta,realUpperDelta,mechLowerDelta,realLowerDelta,collision1,pCollision2,collision3,...
    numLeadToOpen,doubleCase,nBoundary,inevitableCollision] = ...
    plotBiopsyTarget_Sim(p_c,cupSize,needleType,needleGauge,u,v,nGridOpening,robotData,holderData,holderModel,breastHolderRotationAngle);

% Correcting type 2 collision by rotating a little angle in CCW direction
if pCollision2
    breastHolderRotationAngle = 20;%sexagesimal
    [MidSegment,alpha90,mechUpperDelta,realUpperDelta,mechLowerDelta,realLowerDelta,collision1,collision2,collision3,...
        numLeadToOpen,doubleCase,nBoundary,inevitableCollision] = ...
        plotBiopsyTarget_Sim(p_c,cupSize,needleType,needleGauge,u,v,nGridOpening,robotData,holderData,holderModel,breastHolderRotationAngle);
    wasRotated = 1;
end

%% Step 3: Finding a case for needle insertion
% Note: From robot base frame or {s}
% if angle>0, alpha goes in CCW
% else angle<0, alpha goes in CW
%   angle = 0;% Using Shortest distance  CTE
%   angle = (maxAngle + minAngle)/2 is computed middle angle for possible insertion
optionInsertion = 1;% The upper interval by default
angle=0;
if ~collision1 && doubleCase
    inputAngleCase = 1;
    if isequal(optionInsertion,1)% Upper insertion side
        maxAngle = 0;
        minAngle = -mechUpperDelta;
    else
        maxAngle = mechLowerDelta;
        minAngle = 0;
    end
    angle = (maxAngle + minAngle)/2;
elseif collision1 && doubleCase
    inputAngleCase = 2;
    if isequal(optionInsertion,1)%Upper insertion side
        maxAngle = 0;
        minAngle = -mechUpperDelta;
    else
        maxAngle = mechLowerDelta;
        minAngle = 0;
    end
    angle = (maxAngle + minAngle)/2;
elseif ~collision1 && ~doubleCase
    inputAngleCase = 3;
    angle = 0;
else% collision1==True && DoubleCase == False
    if  ~isequal(inevitableCollision,1)
        inputAngleCase = 4;
        if isequal(MidSegment,1) || isequal(inevitableCollision,2)
            maxAngle = mechLowerDelta;
            minAngle = 0;
            optionInsertion = 2;%The lower interval
        else %isequal(MidSegment,7) || isequal(inevitableCollision,3)
            maxAngle = 0;
            minAngle = -mechUpperDelta;%mechLowerDelta
            optionInsertion = 1;%The upper interval
        end
        angle = (maxAngle + minAngle)/2;
    elseif isequal(inevitableCollision,1)
        % Target can't be reached due to inevitable collision. The target position is too near to the
        % internal side of breast holder
        inputAngleCase = 5;
        %angle=0;
    end
end

%% Step 4: Selecting the interval for needle insertion
%Here there's no a selection of insertion interval. If needed, the upper
%one is always prefered.

%% Step 5: Computing new alpha angle for needle insertion
iCollision1 = false;
%if angle>0, alpha goes in CCW
%else angle<0, alpha goes in CW
if isequal(inputAngleCase,1) || isequal(inputAngleCase,2)
    if isequal(optionInsertion,1)
        alpha_old = acos(u'*nBoundary(2,:)');
    else%optionInsertion == 2
        alpha_old = acos(u'*nBoundary(3,:)');
    end
    alpha = alpha_old + angle;
elseif isequal(inputAngleCase,3)
    alpha = alpha90 + angle;
elseif isequal(inputAngleCase,4)
    if isequal(MidSegment,1) || isequal(inevitableCollision,2)
        alpha_old = acos(u'*nBoundary(3,:)');
        alpha = alpha_old + angle;
    else%isequal(MidSegment,7) || isequal(inevitableCollision,3)
        alpha_old = acos(u'*nBoundary(2,:)');
        alpha = alpha_old + angle;
    end
else%inevitableCollision==1
    iCollision1 = true;
    alpha = alpha90 + angle;
end

%% Step 6 : Computing points in breast holder profile at outer surface
[a_c,n_c,needleDistInsertion] = setInsertionPoint_Sim(p_c,u,v,alpha,phi,c_ext);

%% Step 7 : Insertion point at a safety distance from holder
h_c = a_c + offsetPrePlacement*n_c;

%% Step 8: Computing position and direction vectors to C.S. robot
[p_s,n_s,h_s] = getRoboticBiopsyData_Sim(p_c,n_c,h_c,robotData);

%% Step 9: Computing Reachability of target by evaluating robot joint limits
% Notice that n_s == n90
[isReachable,isJointLimit] = isReacheablePoint_Sim(h_s,n_s,collision2,iCollision1,collision3,robotData);

%% Step 10: Computing needle device information
[needleLength,rNeedle,needleThrow,needleDeadSpace,needleDeviceLength] = setNeedleDevice_Sim(p_s,h_s,needleGauge,needleType);

end