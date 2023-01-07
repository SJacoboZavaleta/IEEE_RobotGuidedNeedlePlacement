function loadingBiopsyDevice_App(robot,deviceType,Rneedle_max,Lneedle)
% robot: Rigid body tree model for a robot in Matlab
% deviceType: Needle radius in mm
% Rneedle_max: Needle radius in mm
% Lneedle: Needle length in mm
% ------------------------------------------------------------------
%% Getting Data
mainPath = getRootDirectory;
switch deviceType
    case 'needleFNA'
        needlePath = strcat(mainPath,'\CAD_models\Devices\needleTipo1\meshes\needleBody.STL');
    case 'needleCNB1'
        needlePath = strcat(mainPath,'\CAD_models\Devices\needleTipo2\coreNeedle\meshes\needleCN.STL');
    case 'needleCNB2'
        needlePath = strcat(mainPath,'\CAD_models\Devices\needleTipo3\coreNeedle2\meshes\base_link.STL');
    otherwise
        warning("Bioppsy Device not found ");
end

%% Setting dimensions in m and kg
thicknessArmHolder = 3.18/1000;%1/8 inch
thickessNeedleGuide = 2.29/1000;%0.09 inch
Lneedle = Lneedle/1000;%m
Lneedle_max = 2*Lneedle;%collision origen is at half of cillinder body
Rneedle_max = Rneedle_max/1000;%m
needleBody = robotics.RigidBody('tool');
needleBody.Mass = 0.0099487;
needleBody.CenterOfMass = [9.2519E-12  4.5308E-09 -0.029936];
needleBody.Inertia = [3.497E-06 3.497E-06 3.028E-07 -4.9062E-12 -8.6895E-15 1.8157E-17];%[Ixx Iyy Izz Iyz Ixz Ixy] in kg/m2
addCollision(needleBody,collisionCylinder(Rneedle_max,Lneedle_max))

%% Attaching a needle to the robot
offsetNeedle = -(thicknessArmHolder+thickessNeedleGuide);
T = trvec2tform([0 0 offsetNeedle]);
setFixedTransform(needleBody.Joint,T);
addVisual(needleBody,"Mesh",needlePath,T);
try %If tool already exists, so:
    getBody(robot,'tool');
    replaceBody(robot,'tool',needleBody);
catch %For first time
    addBody(robot,needleBody,'link5');
end

end