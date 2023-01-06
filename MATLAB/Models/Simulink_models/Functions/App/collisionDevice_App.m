function [collision3,needleDeviceLength,sepdist] = collisionDevice_App(p_c,n_c,v,insertionDistance,needleGauge,needleType,needleThrow,needleDeadSpace,Rholder)
%% Checking the needle-breast holder collision.
% INPUTS
% p_c: Biopsy target position from the C.S. {C} in mm
% n_c: Unit vector for needle insertion direction
% v: Unit vector along breast holder surface
% insertionDistance : Distance from the outer surface of breast holder device to biopsy target
% needleGauge: Needle caliber
% needleType: Fine needle or core needle
% needleThrow: Throw distance for the sampling notch in core needles
% needleDeadSpace: Needle tip length in core needles
% Rholder: Major radius for breast holder device
% OUTPUTS
% collision3: If there is collision between the chest wall and needle tip
% needleDeviceLength : Needle device length 
% sepdist: The minimal discante between chest wall and needle tip
% ------------------------------------
%% Needle collision dimensions
% Neede radius for different CAD models (in meters)
switch needleGauge
    case 11
        needleR = 3.404/2/1000;
    case 14
        needleR = 2.413/2/1000;
    case 18
        needleR = 1.27/2/1000;
    case 20
        needleR = 0.908/2/1000;
    case 21
        needleR = 0.819/2/1000;
    otherwise
        needleR = 1/2/1000;
end

%% Some unit convertions
% In meters
insertionDistance = insertionDistance/1000;
needleThrow = needleThrow/1000;
needleDeadSpace = needleDeadSpace/1000;

%% Selecting fixed needle lengths
if strcmp(needleType,'needleFNA')%FNA
    needleDeviceLength = 60;% In mm
elseif strcmp(needleType, 'needleCNB1')%CN type 1
    needleDeviceLength = 82.63;
else% CN type 2
    needleDeviceLength = 138;
end

%% Needle model
offsetNeedle = needleThrow + needleDeadSpace;
needle = collisionCylinder(needleR, 2*(insertionDistance+offsetNeedle));

% Changing its orientation according to S.C. of breast CAD model
RplotTocad = [1 0 0;0 -1 0;0 0 -1];

%% Biopsy target
v = -RplotTocad*v;
nc = RplotTocad*n_c;
nc = -nc;
ny = cross(nc,v);
pc = RplotTocad*p_c/1000;

pNeedle = pc - nc*(insertionDistance+needleThrow/2);
% Needle Pose Transformation
Tneedle = [v ny nc pNeedle; 
           0 0 0 1];
needle.Pose = Tneedle;

%% Chest wall model
% Considering a thickeness of 2 mm.
chestWall = collisionCylinder(Rholder/1000,2/1000);% A thin layer 1mm
chestWall.Pose = [1 0 0 0;
                  0 1 0 0;
                  0 0 1 0;
                  0 0 0 1];

%% Finding type-4 collision
[collision3,sepdist] = checkCollision(chestWall, needle);
% If there's collision shows CAD elements
if collision3
    figure;
    %show(breastHolder);
    %hold on;
    show(needle);
    hold on;
    show(chestWall);
    view([-1 0 0]);
end
end