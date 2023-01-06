function [gridPointInt,gridPointOut,nGridRow] = plotHolderProfile_Sim(n90,u,w,nGridOpening,holderData)
%% Plotting Breast Holder Device Profile
% Function used by Biopsy GUI
% --------------------------------------------------------------
% INPUTS
% ax : Axes for plotting
% holderData : Measurements of each Breast Holder Device
% n90 : Needle Insertion in 
% u,w : Auxiliar vectors 
% nGridOpening : Number of Breast Holder Device openings 
% cupSize : Size or cup of Breast Holder Device 
% OUTPUTS
% gridPointInt : Breast Holder position of outer surface vertex
% gridPointOut : Breast Holder position of  surface vertex
% nGridRow : Number of segments or rows for possible insertions
% --------------------------------------------------------------
%% Step 0: Uploading basic Measurements
R_holderUpper = holderData.R_upper;
T_holder = holderData.T_holder;
g = holderData.g;

%% Step 1: Unit vector w along the generatrix holderData side on radial plane
pIntUpperLimit = [0 0 0]' + u*R_holderUpper;%Upper Internal cone border

%% Step 2 : Dimensions of Cone grids
%nGridOpening = 3 by dafault
nGridRow = nGridOpening*2+2;
upperGridThick = 10;%thickness for first vertical divider holderData
midGridThick = 5;%lateral thickness for the next
heightGrid = (g - (nGridOpening+1)*midGridThick)/nGridOpening;%Supposing the same thickness for grid dividers
%infThickGrid = 5; is the same as midGridThick

%% Step 3: Points of cone radial plane
gridPointInt = zeros(nGridRow,3);
gridPointOut = gridPointInt;
gridPointInt(1,:) = pIntUpperLimit;%First point is the inside upper limit edge
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
gridPointOut(1:end,:) = gridPointInt(1:end,:) + T_holder*n90';

end