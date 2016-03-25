% EE553 Term Project
% Main reference is:
% Power Allocation Strategies for Target Localization in Distributed
% Multiple-Radar Architectures
% Omer CAYIR
% 29.01.2015

% Inputs:
% h: channel type
% M: the number of transmit radars
% N: the number of receive radars

% Outputs:
% H: RCS matrix for channels
function H = getRCS(h,M,N)
H =  ones(M,N); % default
if (h==2 && M==5 && N==7)
    H = [1    1    1    1     1     1     1
         0.01 0.05 0.01 0.022 0.092 0.092 0.092
         0.45 0.35 0.48 0.32  0.49  0.49  0.49
         0.22 0.55 0.55 0.48  0.57  0.57  0.57
         1    1    1    1     1     1     1];
elseif (h==3 && M==5 && N==7)
    H = [0.1  0.05 0.01 0.12 0.09 0.2 0.19
         1    1    1    1    1    1   1
         1    1    1    1    1    1   1
         1    1    1    1    1    1   1
         0.75 0.4  0.45 0.55 0.3  0.2 0.25];
end