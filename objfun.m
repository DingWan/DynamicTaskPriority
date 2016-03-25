% EE553 Term Project
% Main reference is:
% Power Allocation Strategies for Target Localization in Distributed
% Multiple-Radar Architectures
% Omer CAYIR
% 29.01.2015

% Objective function for minimization
% Input:
% P: Mx1 allocated power vector for tranmit radars

% Global Inputs:
% Ae and be: necessary parameters for function evaluations

% Output
% sgm: MSE for target localization
function sgm = objfun(q)
global qmax qmin;

sgm = (qmax(1)-qmin(1))^2/((qmax(1) -q(1))*(q(1) -qmin(1))) + ...
          (qmax(2)-qmin(2))^2/((qmax(2) -q(2))*(q(2) -qmin(2))) + ...
          (qmax(3)-qmin(3))^2/((qmax(3) -q(3))*(q(3) -qmin(3)));