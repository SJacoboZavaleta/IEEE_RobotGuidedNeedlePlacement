function [NeedleLength,rNeedle,NeedleThrow,NeedleDeadSpace,NeedleDeviceLength] = setNeedleDevice_Sim(p_s,h_s,needleGauge,needleType)
%% Setting the needele device's configuration 
% INPUTS
% NeedleLength: Needle length
% rNeedle: Needle radius
% NeedleThrow: Throw distance for the sampling notch in core needles
% NeedleDeadSpace: Needle tip length in core needles
% NeedleDeviceLength: Needle device length 
% OUTPUTS
% p_s: Biopsy target position in frame {s}
% h_s: Final end effector position after preplacement stage in frame {s]
% needleGauge: needle gauge or caliber
% needleType: needle type (FNA or CN)
% --------------------------------------------------------------
%% Definying needle lengths
minDist = norm(p_s-h_s);
needleDistances = [50,60,80,100,120,150,200,250,300];%comertial lengths
aux = needleDistances(minDist<needleDistances);
NeedleLength = aux(1);

%% Selecting needle gauge
switch needleGauge
    case 11
        rNeedle = 3.404/2;%Radius in mm
        %rInnerNeedle = 2.413/2;% minus 2 gauge dimension
        %tNeedle = 0.33;%Thickness in mm
    case 14
        rNeedle = 2.413/2;
        %rInnerNeedle = 1.651/2;
        %tNeedle = 0.254;
    case 18
        rNeedle = 1.27/2;
        %rInnerNeedle = 0.908/2;
        %tNeedle = 0.216;
    case 20
        rNeedle = 0.908/2;
        %rInnerNeedle = 0.718/2;
        %tNeedle = 0.152;
    case 21
        rNeedle = 0.819/2;
        %rInnerNeedle = 0;
        %tNeedle = 0.152;
    otherwise
        rNeedle = 1/2;
        %rInnerNeedle = 0;
        %tNeedle = 0.152;
end

%% Selecting needle type
switch needleType
    case 1%'needleFNA'
        NeedleThrow = 0;
        NeedleDeadSpace = 0;
        NeedleDeviceLength = 60;
    case 2%'needleCNB'
        NeedleThrow = 10;%20 By default
        NeedleDeadSpace = 8;%cte
        NeedleDeviceLength = 83;
    case 3%'needleVAB' Not implemented
        NeedleThrow = 0;
        NeedleDeadSpace = 0;
        NeedleDeviceLength = 138;
end
end