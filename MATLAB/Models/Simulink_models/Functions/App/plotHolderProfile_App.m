function [gridPointInt,gridPointOut,nGridRow] = plotHolderProfile_App(ax,n90,u,w,nGridOpening,cupSize,holderData)
%% Plotting Breast Holder Device Profile
% Function used by Biopsy GUI
% INPUTS
% ax : Axes for plotting
% n90 : Needle Insertion in 
% u,w : Auxiliar vectors 
% nGridOpening : Number of Breast Holder Device openings 
% cupSize : Size or cup of Breast Holder Device
% holderData : Measurements of each Breast Holder Device
% OUTPUTS
% gridPointInt : Breast Holder position of outer surface vertex
% gridPointOut : Breast Holder position of  surface vertex
% nGridRow : Number of segments or rows for possible insertions
% --------------------------------------------------------------
%% Step 0: Uploading basic Measurements
R_holderUpper = holderData.R_upper;
H_holder = holderData.H_holder;
phi = holderData.phi;
T_holder = holderData.T_holder;
g = holderData.g;

%% Step 1: Unit vector w along the generatrix holder side on radial plane
pIntUpperLimit = [0 0 0]' + u*R_holderUpper;%Upper Internal cone border

%% Step 2 : Dimensions of cone grids
%nGridOpening = 3 by dafault
nGridRow = nGridOpening*2+2;
upperGridThick = 10;%thickness for first vertical divider holder
midGridThick = 5;%lateral thickness for the next
heightGrid = (g - (nGridOpening+1)*midGridThick)/nGridOpening;
%infThickGrid = midGridThick;

%% Step 3: Points of cone radial plane
gridPointInt = zeros(nGridRow,3);
gridPointOut = gridPointInt;
gridPointInt(1,:) = pIntUpperLimit';
for i=2:nGridRow
    if mod(i,2)==0
        if i==2
            aux = upperGridThick;
        else
            aux = midGridThick;
        end
    else
        if i==3
            aux= midGridThick + heightGrid - upperGridThick;
        else
            aux = heightGrid;
        end
    end
    gridPointInt(i,:) = gridPointInt(i-1,:) + aux*w';
end
gridPointOut(1:end,:) = gridPointInt(1:end,:) + T_holder*n90';%re1

%% Step 4: Plotting the half part of the holder profile
coneProfile = [gridPointInt(1,:);gridPointOut(1,:);gridPointOut(end,:);[0,0,H_holder+T_holder*cos(phi)];...
    [0,0,H_holder];gridPointInt(end,:);gridPointInt(1,:);[0,0,0];[0,0,H_holder]];
plot3(ax,coneProfile(:,1),coneProfile(:,2),coneProfile(:,3),'Color','blue');
hold(ax,'on');

% Plotting holder dividers
for k=2:nGridRow
    pointInicial = gridPointInt(k,:);
    pointFinal = gridPointOut(k,:);
    line(ax,'XData',[pointInicial(1) pointFinal(1)],'YData',...
        [pointInicial(2) pointFinal(2)],'ZData',...
        [pointInicial(3) pointFinal(3)],'Color','black');
end

if isequal(cupSize,'A')
    color = [1 0.6 1];
elseif isequal(cupSize,'B')
    color = 'c';
elseif isequal(cupSize,'C')
    color = [1 0.8 0.6];
else
    color = 'g';
end

%% Step 5: Getting a coloured surface of holder profile
for l=1:2:nGridRow-1
    pointSideInt1 = gridPointInt(l,:);
    pointSideInt2 = gridPointInt(l+1,:);
    pointSideExt1 = gridPointOut(l,:);
    pointSideExt2 = gridPointOut(l+1,:);
    xfill = [pointSideInt1(1) ;pointSideInt2(1);pointSideExt2(1) ;pointSideExt1(1)];
    yfill = [pointSideInt1(2) ;pointSideInt2(2);pointSideExt2(2) ;pointSideExt1(2)];
    zfill = [pointSideInt1(3) ;pointSideInt2(3);pointSideExt2(3) ;pointSideExt1(3)];
    fill3(ax,xfill,yfill,zfill,color);

    if l== nGridRow-1
        pointSideInt1 = gridPointInt(l+1,:);
        pointSideInt2 = [0 0 H_holder];
        pointSideExt1 = gridPointOut(l+1,:);
        pointSideExt2 = [0,0,H_holder+T_holder*cos(phi)];
        xfill = [pointSideInt1(1) ;pointSideInt2(1);pointSideExt2(1) ;pointSideExt1(1)];
        yfill = [pointSideInt1(2) ;pointSideInt2(2);pointSideExt2(2) ;pointSideExt1(2)];
        zfill = [pointSideInt1(3) ;pointSideInt2(3);pointSideExt2(3) ;pointSideExt1(3)];
        fill3(ax,xfill,yfill,zfill,color);
    end
end
% Reversing z direction
set(ax,'Zdir','reverse');
end