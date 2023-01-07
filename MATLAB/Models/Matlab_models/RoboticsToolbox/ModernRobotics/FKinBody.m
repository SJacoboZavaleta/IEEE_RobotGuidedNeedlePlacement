function T = FKinBody(q,B,M)
% FKinBody - Forward kinematic in body coordinates {tool frame}
% INPUTS:
% q: List of joint coordinates theta1, ... thetan,
% M: Home configuration of the end-effector to Tst(0),
% B: Joint screw axes in the end-effector frame when the 
%        manipulator is at the home position.
% OUTPUT:
% T(q) in SE(3) representing the end-effector frame when the joints 
% are at the specified coordinates (i.t.o Body Frame).
% From Park1995,Lynch2019

T = M;
n = length(q);
for i = 1:n 
    T = T * MatrixExp6s(B(:, i), q(i));
end

end
