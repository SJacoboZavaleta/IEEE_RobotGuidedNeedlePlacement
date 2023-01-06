%% Sampling a biopsy target population for testing the robot-assited needle placement in Simulink
% Based on factorial-method design and the statistical theory of inference
% for population proportion in order to get reasonable sample of targets to
% simulate in Simulink (high-cost computing).
%
% Factorial design (8 groups of simulation)
%   Independent variables:
%       Cup size: A,B,C and D. (4 levels)
%       Needle type: FNA(Gauge 20) y CN(Gauge 14) (2 levels)
%
% Where:
%   FNA: Fine needle aspiration
%   CN: Core needle
%% Analysing first sampling - For a finite biopsy target population of 1000 vector positions
% MATLAB APPROACH (computeTargetsPopulation.m script)
% Given a medium-size sample
n1 = 1000;

% 0. Loading simulation data
mainPath = pwd;
reachabilityA_FNA = load(strcat(mainPath,"\Results\targetsA_FNA.mat",'isReachable'));
reachabilityA_FNA = reachabilityA_FNA.isReachable;
reachabilityA_CN = load(strcat(mainPath,"\Results\targetsA_CN.mat",'isReachable'));
reachabilityA_CN = reachabilityA_CN.isReachable;
reachabilityB_FNA = load(strcat(mainPath,"\Results\targetsB_FNA.mat",'isReachable'));
reachabilityB_FNA = reachabilityB_FNA.isReachable;
reachabilityB_CN = load(strcat(mainPath,"\Results\targetsB_CN.mat",'isReachable'));
reachabilityB_CN = reachabilityB_CN.isReachable;
reachabilityC_FNA = load(strcat(mainPath,"\Results\targetsC_CN.mat",'isReachable'));
reachabilityC_FNA = reachabilityC_FNA.isReachable;
reachabilityC_CN = load(strcat(mainPath,"\Results\targetsC_FNA.mat",'isReachable'));
reachabilityC_CN = reachabilityC_CN.isReachable;
reachabilityD_FNA = load(strcat(mainPath,"\Results\targetsD_FNA.mat",'isReachable'));
reachabilityD_FNA = reachabilityD_FNA.isReachable;
reachabilityD_CN = load(strcat(mainPath,"\Results\targetsD_CN.mat",'isReachable'));
reachabilityD_CN = reachabilityD_CN.isReachable;

% 1. Calculating the theoretical target reachability of a robot-assisted needle placement
successA_FNA_n1 = sum(reachabilityA_FNA);
successA_CN_n1 = sum(reachabilityA_CN);

successB_FNA_n1 = sum(reachabilityB_FNA);
successB_CN_n1 = sum(reachabilityB_CN);

successC_FNA_n1 = sum(reachabilityC_FNA);
successC_CN_n1 = sum(reachabilityC_CN);

successD_FNA_n1 = sum(reachabilityD_FNA);
successD_CN_n1 = sum(reachabilityD_CN);

% 2. Population parameters of sample n1
pA_FNA = round(successA_FNA_n1/n1,2);
varA_FNA = var(reachabilityA_FNA);
pA_CN = round(successA_CN_n1/n1,2);
varA_CN = var(reachabilityA_CN);
pB_FNA = round(successB_FNA_n1/n1,2);
varB_FNA = var(reachabilityB_FNA);
pB_CN = round(successB_CN_n1/n1,2);
varB_CN = var(reachabilityB_CN);
pC_FNA = round(successC_FNA_n1/n1,2);
varC_FNA = var(reachabilityC_FNA);
pC_CN = round(successC_CN_n1/n1,2);
varC_CN = var(reachabilityC_CN);
pD_FNA = round(successD_FNA_n1/n1,2);
varD_FNA = var(reachabilityD_FNA);
pD_CN = round(successD_CN_n1/n1,2);
varD_CN = var(reachabilityD_CN);

% -----------------
% Suspect: The biopsy prediction success increases along with the breast cup size.
% -----------------

% 2. Estimate Confidence Intervals
% 2.1 Check conditions n1*p>5 and n1*(1-p)>5
% n1*pA_FNA
% n1*(1-pA_FNA)
% n1*pA_CN
% n1*(1-pB_CN)
% n1*pB_FNA
% n1*(1-pB_FNA)
% n1*pB_CN
% n1*(1-pB_CN)
% n1*pC_FNA
% n1*(1-pC_FNA)
% n1*pC_CN
% n1*(1-pC_CN)
% n1*pD_FNA
% n1*(1-pD_FNA)
% n1*pD_CN
% n1*(1-pD_CN)

% 3. General form
Z=1.96;% Using 95% CIs
[ciA_FNA,~] = CI(pA_FNA,Z,n1);
[ciA_CN,~] = CI(pA_CN,Z,n1);
[ciB_FNA,~] = CI(pB_FNA,Z,n1);
[ciB_CN,~] = CI(pB_CN,Z,n1);
[ciC_FNA,~] = CI(pC_FNA,Z,n1);
[ciC_CN,~] = CI(pC_CN,Z,n1);
[ciD_FNA,~] = CI(pD_FNA,Z,n1);
[ciD_CN,~] = CI(pD_CN,Z,n1);

%% SECOND SAMPLING: Find a suitable sample n2 for simscape simulation from sample n1
% Simulink approach
% In this step the sample n1 behaves as a new population for this
% particular sampling

% 1. Needed samples for working in Simulink instead of simulating
% n1 = 1000 experiments per each simulation group 
%Z=1.96;% Using 95% CI
E=0.05;% Maximum error 
N = n1;% new N for sampling n2
d = 0.05;% Maximum error 

% Sampling n2: For Simulink environment
n2_A_FNA=round(sample_n2(N,Z,varA_FNA,d));
n2_A_CN=round(sample_n2(N,Z,varA_CN,d));
n2_B_FNA=round(sample_n2(N,Z,varB_FNA,d));
n2_B_CN=round(sample_n2(N,Z,varB_CN,d));
n2_C_FNA=round(sample_n2(N,Z,varC_FNA,d));
n2_C_CN=round(sample_n2(N,Z,varC_CN,d));
n2_D_FNA=round(sample_n2(N,Z,varD_FNA,d));
n2_D_CN=round(sample_n2(N,Z,varD_CN,d));

% 2. Using the maximum sample for each breast holder size (both FNA and CN)
n2_A = max([n2_A_FNA,n2_A_CN]);
n2_B = max([n2_B_FNA,n2_B_CN]);
n2_C = max([n2_C_FNA,n2_C_CN]);
n2_D = max([n2_D_FNA,n2_D_CN]);
Totalexperiments = (n2_A+n2_B+n2_C+n2_D)*2;

% Showing a summary
t = table([n2_A, n2_A]',[n2_B, n2_B]',[n2_C, n2_C]',[n2_D, n2_D]','VariableNames',{'A','B','C','D'}, 'RowNames',{'FNA','CN'});
disp(t);

function n2 = sample_n2(n1,Z,var,d)
    n2 = 1/((1/n1)+d^2/(Z*var));
end

function [ci,e] = CI(p,Z,n)
    e = Z*sqrt(p*(1-p)/n); 
    ci(1) = p + e;
    ci(2) = p - e;
end