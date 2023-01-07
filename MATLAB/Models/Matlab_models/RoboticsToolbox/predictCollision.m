function [collision2,verticalHolderDivider] = predictCollision(p_c,needleType)
%% Predicting type 2 collision by needle insertion through opened breast holder lid. 
% INPUTS
% p_c : Biopsy target position in frame {c}
% needleType: needle types such as 1 (fine needles) or 2 (core needles)
% OUTPUTS
% collision2 : type 2 collision
% verticalHolderDivider :  Number of vertical supports (in CCW)
% ------------------------------------------------------------------
%% A geometrical approach using angles and lids order.

pos = atan2d(p_c(2),p_c(1));

if needleType==1
    delta = 2.35+0.1;%
else
    delta = 3.03+0.1;
end

l = [0, 45.13, 90.5, 135.88, 360-179.24, 360-134.37, 360-89.5 360-44.62];
angleLimits = [l(1)+delta,...
              l(2)-delta,l(2)+delta,...
              l(3)-delta,l(3)+delta,...
              l(4)-delta,l(4)+delta,...
              l(5)-delta, l(5)+delta,...
              l(6)-delta,l(6)+delta,...
              l(7)-delta,l(7)+delta,...
              l(8)-delta,l(8)+delta,...
              360-delta, 360];

if pos<0
    pos = 360-abs(pos);
end

verticalHolderDivider = find(pos<angleLimits,1,'first');

if rem(verticalHolderDivider,2)==0
    collision2 = false;
else
    if verticalHolderDivider==17 %drops in first vertical support
        verticalHolderDivider = 1;
    else
        verticalHolderDivider = (verticalHolderDivider+1)/2;
    end
    
    collision2 = true;
end
end