function Re = robotRotation(pE,n_s)
%% Calculating rotation matrix of end effector in {e} frame
% INPUTS
% pE : End-effector position from {0}. (3x1 size)
% n_s : Ze direction of end-effector from {0} to target. (3x1 size)
% OUTPUTS
% Re : End-effector rotation from {0} (3x3 size) In SO3
%

%% According to homogenous transformation matrix
% Where 
%   T0e = [Re pE
%          0 0 0 1];
u = [pE(1) pE(2) 0 ]'/norm([pE(1) pE(2) 0 ]);% XoYo proyection
ze = n_s;
xe = cross([0 0 1]',u);
ye = cross(ze,xe);
Re = [xe ye ze];

end

