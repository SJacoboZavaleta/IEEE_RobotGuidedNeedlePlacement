function [Z] = getZ(A)
%% Getting Z direction from a Homegeneous Transformation Matrix
% INPUT
% A: Homegeneous Transformation Matrix (4x4 size)
% OUTPUT
% Z: direction in z axis (3x1 size)
Z = A(1:3,3);
end

