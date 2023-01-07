function  R = MatrixExp3s(omega,q)
% MATRIXEXP3S - Exponencial matrix for Rotations
% in SO(3)
% ----------------------------------------------
% INPUTS:
% omega = [w] from Screw axis S =[w v]
% q : Joint variables theta1 ... thetan (nx1)
% OUTPUTS: 
% R : Rotation matrix (3x3)
% From Lynch2019
% ----------------------------------------------
R = eye(3)+sin(q)*omega+(1 - cos(q))*omega*omega;
end
