function [p_s,a_s,n_s,h_s,eeRPRY] = transformToRobotBase_App(p_c,a_c,n_c,h_c,robotData)
%% Transforming robot from base frame {S} to breast holder {C}
% INPUTS
% p_c: Biopsy target position in {c} frame
% a_c: Vector position in outer surface of breast holder
% n_c: Needle insertion direction in {c} frame  
% h_c: Final end effector position before biopsy in {c} frame 
% robotData: Needed robot datra
% OUTPUTS
% p_s: Biopsy target position in {s} frame
% a_s: Vector position in outer surface of breast holder in {s} frame
% n_s: Needle insertion direction in {s} frame
% h_s: Final end effector position before biopsy in {s} frame
% eeRPRY: Orientation of end effector in RPY coordinates
% --------------------------------------------------------------
%%
Tsc = robotData.Tsc;

p_s = Tsc*[p_c; 1];
p_s = p_s(1:3);

a_s = Tsc*[a_c; 1];
a_s = a_s(1:3);

n_s = -getR(Tsc)*n_c;

h_s = Tsc*[h_c; 1];
h_s = h_s(1:3);

R = robotRotation(h_s,n_s);
eeRPRY = tr2rpy(R);
end