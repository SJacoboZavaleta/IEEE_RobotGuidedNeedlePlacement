function [p_s,n_s,h_s] = getRoboticBiopsyData_Sim(p_c,n_c,h_c,robotData)
%% Convert data got in breast holder S.C. or {C} to robot base frame or {S}
% --------------------------------------------------------------------------

Tsc = robotData.Tsc;

p_s = Tsc*[p_c; 1];
p_s = p_s(1:3);

%a_s = Tsc*[a_c; 1];
%a_s = a_s(1:3);

n_s = -getR(Tsc)*n_c;

h_s = Tsc*[h_c; 1];
h_s = h_s(1:3);

end

