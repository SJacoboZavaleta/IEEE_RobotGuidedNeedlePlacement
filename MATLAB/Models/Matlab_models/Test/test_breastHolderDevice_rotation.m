%% New Breast Holder
% Rigid body tree includes collision mesh too.
% mesh for lead i = breastHolderA.Bodies{1,lead i}.Collisions

mainPath = startupGUI;
holderModel = load(strcat(mainPath,"\Simulink_models\Data\breastHolderModel.mat"));
breastHolderA = holderModel.breastHolderA;
breastHolderA.DataFormat = 'column';

%% Total model
newHolder = rigidBodyTree;
newHolder.DataFormat='column';
table = rigidBody('table');

basename = newHolder.BaseName;
addBody(newHolder,table,basename)
showdetails(newHolder)


baseHolder = breastHolderA.Base;
baseHolder = copy(baseHolder);
jnt1 = rigidBodyJoint('Jholder','revolute');
baseHolder.Joint = jnt1;
addBody(newHolder,baseHolder,'table')
showdetails(newHolder)

addSubtree(newHolder,'base_link',breastHolderA)
showdetails(newHolder)
%%
show(newHolder,'Collisions','on','Visuals','off')
%%
config = homeConfiguration(newHolder);
config(1) = 0;%config(1).JointPosition in DataFormat='struct'
config(2) = pi/2;
show(newHolder,config,'Collisions','on','Visuals','off');
hold on;
view([1 0 0]);

needle = collisionCylinder(8/1000, 20/1000);
box = collisionBox(10/1000,10/1000,2/1000);
show(needle)
show(box)
devices = {needle};

coll = ~checkCollision(newHolder,config,devices);
disp(coll)
%% Only leads
newHolder = rigidBodyTree;
newHolder.DataFormat='column';
table = rigidBody('table');
jnt0 = rigidBodyJoint('Jholder','revolute');
table.Joint = jnt0;

basename = newHolder.BaseName;
addBody(newHolder,table,basename)
showdetails(newHolder)

% baseHolder = breastHolderA.Base;
% baseHolder = copy(baseHolder);
% jnt1 = rigidBodyJoint('Jholder','revolute');
% baseHolder.Joint = jnt1;
% addBody(newHolder,baseHolder,'table')
% showdetails(newHolder)

addSubtree(newHolder,'table',breastHolderA)
showdetails(newHolder)
show(newHolder)

config = homeConfiguration(newHolder);
config(1) = pi/8;%config(1).JointPosition in DataFormat='struct'
config(2) = pi/2;
show(newHolder,config,'Collisions','on','Visuals','off');
view([1 0 0]);