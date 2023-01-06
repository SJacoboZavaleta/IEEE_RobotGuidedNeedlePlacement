function Re = robotRotation_App(pE,ns)
%% Getting the rotation matrix of end effector
% INPUT
% pE: End-effector position from {0}. (3x1 size)
% ns: Ze direction of end-effector from {0} to target. (3x1 size)
% OUTPUT
% Re: End effector rotation from {0} 3x3 in SO3
% --------------------------------------------------------------
%% According to homogenous transformation matrix
%   T0e = [Re pE
%         0 0 0 1];

u = [pE(1) pE(2) 0 ]'/norm([pE(1) pE(2) 0 ]);% XoYo proyection
ze = ns;
xe = cross([0 0 1]',u);
ye = cross(ze,xe);
Re = [xe ye ze];

end

