% EE553 Term Project
% Main reference is:
% Power Allocation Strategies for Target Localization in Distributed
% Multiple-Radar Architectures
% Omer CAYIR
% 29.01.2015

% Computes the gradient of objective function
% Inputs:
% P: Mx1 allocated power vector for tranmit radars
% grad: gradient computation is (1:analytically, 2:numerically)

% Global Inputs:
% Ae and be: necessary parameters for function evaluations

% Output
% gradf: the gradient of objective function at P
function gradf = objfungrad(q,grad)
global qmax qmin;
if (nargin<2)
    grad = 1;
end
if (grad==2)
    display('Numerical Method is not considered here.');
else
    gradf = [((qmax(1) - qmin(1))^2*(2*q(1)-qmax(1)- qmin(1)))/(((qmax(1) -q(1))*(q(1) - qmin(1)))^2)
             ((qmax(2) - qmin(2))^2*(2*q(2)-qmax(2)- qmin(2)))/(((qmax(2) -q(2))*(q(2) - qmin(2)))^2)
             ((qmax(3) - qmin(3))^2*(2*q(3)-qmax(3)- qmin(3)))/(((qmax(3) -q(3))*(q(3) - qmin(3)))^2)];
end