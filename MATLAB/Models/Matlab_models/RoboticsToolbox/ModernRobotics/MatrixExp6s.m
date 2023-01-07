function T = MatrixExp6s(S,q)
% MATRIXEXP6S - Exponencial matrix for Homogeneous
% Transformation matrix in SE(3)
% ----------------------------------------------
% INPUTS:
% S : Spatial Screw axis S1 ... Sn (6xn)
% q : Joint variables theta1 ... thetan (nx1)
% OUTPUT:
% T : Homogeneous Transfomation Matrix (4x4)
% From Lynch2019
% ------------------------------------------------
w = S(1:3); % if S = [w v]' --> Getting only w
omega = VecToso3(w);% [w]
v = S(4:6);

T = [MatrixExp3s(omega,q), ... % [w*q]
    (eye(3)*q + (1 - cos(q))*omega ...
    + (q - sin(q))*omega*omega)* v;
    0, 0, 0, 1];
end
