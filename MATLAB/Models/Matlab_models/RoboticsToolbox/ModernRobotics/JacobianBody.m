function Jb = JacobianBody(q,S,Mst)
% JACOBIANBODY - Body jacobian of robot at home position
% from {n+1} or end-effector frame. 
% Numerical Solution
% --------------------------------------------------------------
% INPUTS
% q : Joint variables theta1 ... thetan (nx1)
% S : Spatial joint Screw axis S1 ... Sn (6xn)
% Mst : Home position matrix of frame from {t} to {s}
% OUTPUT :
% Jb : Body Jacobian (6xn)
% -------------------------------------------------------------
% From Murray1995: Using screw axis S directly
%--------------------------------------------------------------
n = length(q);
Jb = zeros(6,n);
T = TransInv(Mst);
for i = n:-1:1 
    T = T * MatrixExp6s(S(:,i),-q(i));
	Jb(:, i) = Adjoint(T) * S(:,i);
end
end
