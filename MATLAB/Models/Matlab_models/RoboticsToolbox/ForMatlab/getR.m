function [R] = getR(A)
%% Getting rotation matrix from a Homegeneous Transformation Matrix
% INPUT
% A: Homegeneous Transformation Matrix (4x4 size)
% OUTPUT
% R: rotation matrix (3x3 size)
R = A(1:3,1:3);
end

