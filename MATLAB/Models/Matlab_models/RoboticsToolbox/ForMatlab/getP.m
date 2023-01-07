function P = getP(A)
%% Getting vector position from a Homegeneous Transformation Matrix
% INPUT
% A: Homegeneous Transformation Matrix (4x4 size)
% OUTPUT
% P: vector position (3x1 size)
P = A(1:3,4);
end

