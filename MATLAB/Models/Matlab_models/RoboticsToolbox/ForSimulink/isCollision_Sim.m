function [collision1,collision2,collision3,numLidToOpen] = isCollision_Sim(p_c,MidSegment,needleL,nc,v,needleType,needleGauge,Rholder,handRotation)
% IS COLLISION - Checking the needle-breast holder collision.
% INPUTS:
% p_c: Biopsy target position in {C} frame
% MidSegment: Breast holder dividers
% needleL: A determined needle length in mm
% nc: Shortest needle insertion unit vector
% v: Orthogonal unit vector to u.
% needleType: Needle type (FNA and CN)
% needleGauge: Needle gauge 
% Rholder: Radius holder
% handRotation: If a rotation of breast holder is needed to avoid type 2 collision 
% OUTPUTS:
% collision1, collision2, collision3: Collision types
% numLidToOpen: Lid to be opened for needle insertion
% ------------------------------------------------------------------
%% Step1 : Collision type 1 for breast holder dividers (rows). This can be evitable.
if mod(MidSegment,2) == 0% odd number
    collision1 = false;
else
    collision1 = true;
end

%% Step 2: Collision type 2 for breast holder dividers (columns). This can not be
% inevitable.
% Loading breast holder models

if strcmp(holderSize,'A')
    breastHolder = holderModel.breastHolderA;
elseif strcmp(holderSize,'B')
    breastHolder = holderModel.breastHolderB;
elseif strcmp(holderSize,'C')
    breastHolder = holderModel.breastHolderC;
else
    breastHolder = holderModel.breastHolderD;
end

% Modifying breast holder model for a new rotable model
breastHolder.DataFormat = 'column';
holderLids = rigidBodyTree;
holderLids.DataFormat = 'column';

% For a total model
newHolder = rigidBodyTree;
newHolder.DataFormat='column';
table = rigidBody('table');

basename = newHolder.BaseName;
addBody(newHolder,table,basename);

baseHolder = breastHolder.Base;
baseHolder = copy(baseHolder);
jnt1 = rigidBodyJoint('Jholder','revolute');
baseHolder.Joint = jnt1;
addBody(newHolder,baseHolder,'table');
%showdetails(newHolder);
addSubtree(newHolder,'base_link',breastHolder);

% For a only leads model
holderLids.DataFormat='column';
table2 = copy(table);
jnt0 = rigidBodyJoint('Jholder2','revolute');
table2.Joint = jnt0;

basename2 = holderLids.BaseName;
addBody(holderLids,table2,basename2);

addSubtree(holderLids,'table',breastHolder);
%showdetails(holderLids);

% Finally, reemplacing a desired rotable breast holder model
breastHolder = newHolder;

numLids = 8;%holderCollision.NumBodies;

% Needle collision dimensions
switch needleGauge
    case 11
        needleR = 3.404/2000;%Radius in m
    case 14
        needleR = 2.413/2000;
    case 18
        needleR = 1.27/2000;
    case 20
        needleR = 0.908/2000;
    case 21
        needleR = 0.819/2000;
    otherwise
        needleR = 1/2000;
end

% Data by default
% needleL is the shortest distance from pc to holder's inner surface
needleL = needleL/1000;%Approach: Using shortest distance
if needleType == 1%FNA
    needleThrow = 0;
    needleDeadSpace = 0;
elseif needleType == 2%CN type 1
    needleThrow = 10/1000;
    needleDeadSpace = 8/1000;
else% CN type 2
    needleThrow = 20/1000;%
    needleDeadSpace = 8/1000;
end

%Using an offset in case of core needle biopsy: To verify collision
if needleType==1
    offsetNeedle = 0;
else
    offsetNeedle = needleThrow + needleDeadSpace;%considering post-fire
end
needle = collisionCylinder(needleR, 2*(needleL+offsetNeedle));

% Changing the orientation according to S.C. of CAD
RplotTocad = [1 0 0;0 -1 0;0 0 -1];

% Biopsy point data
v = -RplotTocad*v;
nc = RplotTocad*nc;%from target to exterior
nc = -nc;%from exterior to inside breast
ny = cross(nc,v);
pc = RplotTocad*p_c/1000;

pNeedle = pc - nc*(needleL);

% Needle Pose Transformation
Tneedle = [v ny nc pNeedle; 
           0 0 0 1];
needle.Pose = Tneedle;

% numLids is an even number
positionApproach = linspace(0,2*pi,numLids+1);
angle = atan2(pc(2),pc(1));

if angle<0
    angle = 2*pi + angle;
end

numLidToOpen = find(positionApproach>angle);
numLidToOpen = numLidToOpen(1)-1;

%% Predicting collision 2
[collision2predict,~] = predictCollision(pc,needleType);%It is also used in isCollision_App
%% Plotting breast holder device
%config1 = breastHolder.homeConfiguration;%Home position: Not shown here
config2 = holderLids.homeConfiguration;%Home position
needleDevice = {needle};
% New configuration by a needed rotation along Zc in CCW
if handRotation~=0
    config2(1) = handRotation*pi/180;

    collision2 = checkCollision(holderLids,config2,needleDevice);
    collision2 = ~collision2(2);
else
    collision2 = checkCollision(holderLids,config2,needleDevice);
    collision2 = ~collision2(2) || collision2predict;
end


%% Step 3: For collision 3: With a mesh simulating a woman chest wall
chestWall = collisionCylinder(Rholder/1000,2/1000);% a thin layer 2mm
chestWall.Pose = [1 0 0 0;
                  0 1 0 0;
                  0 0 1 0;
                  0 0 0 1];
collision3 = checkCollision(chestWall, needle);
end