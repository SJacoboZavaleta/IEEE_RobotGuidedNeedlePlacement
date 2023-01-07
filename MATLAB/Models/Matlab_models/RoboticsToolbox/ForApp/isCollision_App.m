function [collision1,collision2,collision3,numLidToOpen] = isCollision_App(p_c,MidSegment,needleL,n_c,v,holderSize,needleType,Rholder,holderModel,handRotation)
%% Checking the needle-breast holder collision
% INPUTS
% p_c: Biopsy target point from C.S. {C} in mm
% MidSegment: Breast holder dividers
% needleL: Shortest distance from p_c to holder's inner surface in mm
% n_c: Shortest needle insertion unit vector
% v: orthogonal unit vector to u
% OUTPUTS
% collision1, collision2, collision3: Collision types
% numLidToOpen: Lid to be opened
% ------------------------------------------------------------------
%% Step1 : Type 1 collision for breast holder dividers (rows). This can be evitable.
if mod(MidSegment,2) == 0% odd
    collision1 = false;
else
    collision1 = true;
end

disp(collision1);
%% Implementing breast holder and needle models - Building a breast holder model
% Getting holder model (rigid body tree model) and collision mesh for breast model CAD 
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

% For the whole model
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
addSubtree(newHolder,'base_link',breastHolder);
%showdetails(newHolder);

% For only a lid model
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

% Building a needle model
numLids = 8;% Number of lids for breast holder

% Needle collision dimensions
needleL = needleL/1000;%Approach: Using shortest distance
if needleType == 1%FNA
    needleR = 0.908/2/1000;% gauge 20. needle radius possible
    needleThrow = 0;
    needleDeadSpace = 0;
elseif needleType == 2%CNB1
    needleR = 2.413/2/1000;% gauge 14. needle radius possible
    needleThrow = 10/1000;
    needleDeadSpace = 8/1000;
else%CNB2
    needleThrow = 20/1000;%
    needleDeadSpace = 8/1000;
end

% Calculating an offset distance for CN needles
if needleType==1
    offsetNeedle = 0;
else
    %Considering post-fire of needle device
    offsetNeedle = needleThrow + needleDeadSpace;
end

needle = collisionCylinder(needleR, 2*(needleL+offsetNeedle));

%% Step 2: Collision type 2 for breast holder dividers (columns).
% Changing the orientation according to S.C. of breast holder CAD
RplotTocad = [1 0 0;0 -1 0;0 0 -1];

% Biopsy point data
v = -RplotTocad*v;
n_c = RplotTocad*n_c;
n_c = -n_c;
ny = cross(n_c,v);
p_c = RplotTocad*p_c/1000;

pNeedle = p_c - n_c*(needleL);
% Needle Pose Transformation
Tneedle = [v ny n_c pNeedle; 
           0 0 0 1];
needle.Pose = Tneedle;

positionApproach = linspace(0,2*pi,numLids+1);

angle = atan2(p_c(2),p_c(1));
if angle<0
    angle = 2*pi + angle;
end

numLidToOpen = find(positionApproach>angle);
numLidToOpen = numLidToOpen(1)-1;
%% Predicting collision 2
[coll2_prediction,~] = predictCollision(p_c,needleType);
disp("prediction for collision 2");
disp(coll2_prediction);
%% Plotting breast holder device
close all
f = figure('Name','Breast Holder Device','NumberTitle','off');
ax = axes(f);

config1 = breastHolder.homeConfiguration;%Home position
config2 = holderLids.homeConfiguration;%Home position
needleDevice = {needle};

% Using a needed rotation of breast holder along Zc in CCW
if handRotation~=0
    config2(1) = handRotation*pi/180;%User rotation
    config1 = config2;%Coying the same rotation for completed model to show
    config1(numLidToOpen+1) = pi/2;%After rotation, lid is opened.
    
    %When checkCollision is false, the needle is in collision with the
    %holder structure
    collision2 = checkCollision(holderLids,config2,needleDevice);
    collision2 = ~collision2(2);
    disp("Initial collision 2");
    disp(~collision2(2));
else
    config1(numLidToOpen+1) = pi/2;
    
    collision2 = checkCollision(holderLids,config2,needleDevice);
    disp("Collision 2");
    disp(~collision2(2));
    collision2 = ~collision2(2) || coll2_prediction;
end

disp("Lid ");
disp(numLidToOpen);
disp("Final collision 2");
disp(collision2);

holderAx = show(breastHolder,config1,'Parent',ax,'Frames','off');
hold(ax,'on');
holderAx.CameraPositionMode = 'auto';
xlim(ax,[-0.15 0.15]), ylim(ax,[-0.15 0.15]), zlim(ax,[-0.1 0.01])%in meters

[~,patchObj] = show(needle,'Parent',ax);
patchObj.FaceColor = [0 1 1];%color green
view(ax,-n_c);

f2 = figure;
ax2 = axes(f2);
holderAx2 = show(holderLids,config2,'Parent',ax2,'Frames','off','Collisions','on','Visuals','off');
hold(ax2,'on');
holderAx2.CameraPositionMode = 'auto';
xlim(ax2,[-0.15 0.15]), ylim(ax2,[-0.15 0.15]), zlim(ax2,[-0.1 0.01])%in meters

show(needle,'Parent',ax2);
view(ax2,-n_c);

%% OLD collision between needle and neighbour lids.

%% Step 3 for collision 3: With a mesh simulating a woman chest wall
chestWall = collisionCylinder(Rholder/1000,1/1000);% a thin layer 1mm
chestWall.Pose = [1 0 0 0;
                  0 1 0 0;
                  0 0 1 0;
                  0 0 0 1];

collision3 = checkCollision(chestWall,needle);
show(chestWall);
disp("col3")
disp(collision3);

end