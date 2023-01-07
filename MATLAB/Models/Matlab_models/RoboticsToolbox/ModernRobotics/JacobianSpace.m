function Js = JacobianSpace(S,q)
% JACOBIANSPACE - Jacobian of robot from {0} at home position
% INPUTS:
% S : Spatial Screw axis S1 ... Sn (6xn)
% q: A list of joint coordinates.
% OUTPUT:
% Js: Space Jacobian (6xn real numbers).
n = length(q);
Js = S;%Let Js(:,1)=S1
T = eye(4);
for i = 2:n
    T = T * MatrixExp6s(S(:, i - 1),q(i - 1));
	Js(:, i) = Adjoint(T) * S(:, i);
end
end
