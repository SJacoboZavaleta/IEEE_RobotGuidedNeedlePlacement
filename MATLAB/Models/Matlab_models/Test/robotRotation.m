function Re = robotRotation(pE,ns)
%INPUTS:
% pE : end-effector position from {0}. 3x1
% n : Ze direction of end-effector from {0} to target. 3x1
%OUTPUTS:
% Re : end-effector rotation from {0} 3x3 SO3
u = [pE(1) pE(2) 0 ]'/norm([pE(1) pE(2) 0 ]);%Proyection
ze = ns;
xe = cross([0 0 1]',u);%
ye = cross(ze,xe);
Re = [xe ye ze];

%T0e = [Re pE
%       0 0 0 1];

end

