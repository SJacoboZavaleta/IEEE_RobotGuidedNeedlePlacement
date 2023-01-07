function [q,dq,qTimePoints] = getNeedlePath_Sim(q5min,q5max,t0,tf,numPoints)
%% Get trajectory for needle insertion
% The joint trajectory interpolation and type is considered a constant. 
% INPUT
% q5min: minimum distance to reach target in -Z5 of frame {5}. q5min<0
% q5max: maximum distance to reach target in -Z5 of frame {5}. q5max<0
% t0, tf: initial and final time for needle trajectory calculation
% OUTPUTS
% q, dq: joint position and its derivatives in meters
% qTimePoints: Time series for trajectories
% --------------------------------------------------------------------------
%% A simple Time-Scaling approach 
s = linspace(0,1,numPoints);
qTimePoints = linspace(t0,tf,numPoints);

q = q5max + s*abs(q5max-q5min);
dq = zeros(1,numPoints);

end

