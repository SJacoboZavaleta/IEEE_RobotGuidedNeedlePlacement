function T = FKinSpace(q,S,M)
% FKinspace - Forward kinematic in Space coordinate S
% INPUTS:
% q: List of joint coordinates theta1, ... thetan,
% S: The joint screw axes in the space frame when the manipulator
%        is at the home position,
% M: Home configuration of the end-effector to Tst(0).
% OUTPUT :
% T(q) in SE(3) representing the end-effector frame, when the joints 
% are at the specified coordinates (i.t.o Space Frame).
% From Park1995, Lynch2019
T = M;
n = length(q);
for i = n: -1: 1
    T = MatrixExp6s(S(:, i),q(i)) * T;
end

end
