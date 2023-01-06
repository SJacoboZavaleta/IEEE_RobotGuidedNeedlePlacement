
%%
clear, close all
%holderPath = "D:\nuevaTesis2020\DESARROLLO\FUSION360\Exportaciones\Holder\version1\breastHolderACorr_description\urdf\breastHolderACorr.urdf";
holderPath = "D:\nuevaTesis2020\DESARROLLO\SOLIDWORKS\Holders\breastHolderB_supports\urdf\breastHolderB_supports.urdf";
breastHolder = importrobot(holderPath);
breastHolder.Gravity = [0 0 -9.81];
breastHolder.DataFormat = 'column';
%%
showdetails(breastHolder)

% Default Collision mesh
ax = show(breastHolder,'visuals','off','collision','on','Frames','off');
hold(ax,'on');
xlabel('x');
ylabel('y');
xlim(ax,[-0.15 0.15]), ylim(ax,[-0.15 0.15]), zlim(ax,[-0.1 0.01])%in meters
%set(ax,'Zdir','reverse');

%% Add Needle body
needleR = 1/1000;
%First approach: Using shortest distance
needleL = 1/1000;
needle = collisionCylinder(needleR, needleL);

% Modifiying S.C
nc = [0 0 -1]';
pc = [0 0 5]'/1000;
%nc(3) = -nc(3);
%ny = cross(nc,v);
%pc(3) = -pc(3);

% Needle Configuration
%Tneedle = [v ny nc pc; 0 0 0 1];
Tneedle = [diag([1 1 1]) pc; 0 0 0 1];
needle.Pose = Tneedle;
[~, patchObj] = needle.show('Parent', ax);
patchObj.FaceColor = [1 0 0];

% I = transl([0 0 0 ]);
% trplot(I);
% hold on;
% trplot(Tneedle);
% hold on;
% plot3(pc(1),pc(2),pc(3),'o');
% q = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 pi/2]';
% ax = show(breastHolder,q,'Frames','off','visuals','off','collision','on');
%%
checkCollision(breastHolder,0,{needle})
 
%%
collisionMeshPath = "D:\nuevaTesis2020\DESARROLLO\SOLIDWORKS\Holders\breastHolderB_supports2\meshes\" ;
holderCollision = importrobot(holderPath,'MeshPath',collisionMeshPath);
holderCollision.DataFormat = 'column';

% Plots the same as above
ax = show(holderCollision,'visuals','off','collision','on','Frames','off');
%%
% For each body, read the corresponding STL file
holderBodies = [{holderCollision.Base} holderCollision.Bodies];
for i = 1:numel(holderBodies)
    if ~isempty(holderBodies{i}.Visuals)
        % Assumes the first Visuals element is the correct one.
        visualDetails = holderBodies{i}.Visuals{1};
        
        % Extract the part of the visual that actually specifies the STL name
        visualParts = strsplit(visualDetails, ':');
        stlFileName = visualParts{2};
        
        % Read the STL file
        stlData = stlread(fullfile(collisionMeshPath, stlFileName));
        
        % Create a collisionMesh object from the vertices (in meters)
        collisionArrayFromMesh{i,1} = collisionMesh(stlData.Points/1000);
        
        % Transform is always identity
        collisionArrayFromMesh{i,2} = eye(4);
    end
end

%%
[collision,~,~] = checkCollision(collisionArrayFromMesh{1,1},needle);

%%% ERROR, EL CUERPO DE COLISION DEL SOPORTE ESTRUCTURAL SIEMPRE ES SIEMPRE
%%% HECHO POR CONVEX, NO DE FORMA ESPECÍFICA PARA CADA SOPORTE VERTICAL
%%% SINO COMO UN CONJUNTO CERRADO.