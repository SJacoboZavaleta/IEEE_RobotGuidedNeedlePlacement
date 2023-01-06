%% BREAST BIOPSY PROJECT

clear
clc
addpath(genpath(pwd));

% Set the simulation cache folder to a work folder
if ~isfolder('work')
    mkdir('work')
end
Simulink.fileGenControl('set','CacheFolder','work')
