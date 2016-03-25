% EE553 Term Project
% Main reference is:
% Power Allocation Strategies for Target Localization in Distributed
% Multiple-Radar Architectures
% Omer CAYIR
% 29.01.2015

% The function to compute necessary parameters for optimization
% Inputs:
% M: the number of transmit radars
% N: the number of receive radars
% tx and ty: position of transmit radars in meters for x-y plane
% rx and ry: position of receive radars in meters for x-y plane
% H: RCS matrix for channels
% trg: 2x1 vector for the position of target in meters

% Global Outputs:
% Ae and be: necessary parameters for function evaluations
function computeAebe(M,N,tx,ty,rx,ry,H,trg)
global Ae be;
alpha = ones(M,N);
beta = 0.2e+6*ones(1,M); % equal bandwith for each transmit radar
x = trg(1);
y = trg(2);
ga = zeros(M,1);
gb = ga;
gc = ga;
c = 3e+8; % the speed of light
sigmaw = 1; % the noise power for target localization
for i=1:M
    rmt = sqrt((tx(i)-x)^2+(ty(i)-y)^2);
    for j=1:N
        rnr = sqrt((rx(j)-x)^2+(ry(j)-y)^2);
        alpha(i,j)=9e+14/(rmt*rnr)^2;
        ga(i) = ga(i)+alpha(i,j)*abs(H(i,j))*((tx(i)-x)/rmt+(rx(j)-x)/rnr)^2;
        gb(i) = gb(i)+alpha(i,j)*abs(H(i,j))*((ty(i)-y)/rmt+(ry(j)-y)/rnr)^2;
        gc(i) = gc(i)+alpha(i,j)*abs(H(i,j))*((tx(i)-x)/rmt+(rx(j)-x)/rnr)* ...
            ((ty(i)-y)/rmt+(ry(j)-y)/rnr);
    end
    ga(i) = ga(i)*(8*pi^2*beta(i)^2)/(sigmaw*c^2);
    gb(i) = gb(i)*(8*pi^2*beta(i)^2)/(sigmaw*c^2);
    gc(i) = gc(i)*(8*pi^2*beta(i)^2)/(sigmaw*c^2);
end
be = ga+gb;
Ae = 0.5*ga*gb'+0.5*gb*ga'-gc*gc';