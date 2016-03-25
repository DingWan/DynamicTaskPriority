% EE553 Term Project
% Main reference is:
% Power Allocation Strategies for Target Localization in Distributed
% Multiple-Radar Architectures
% Omer CAYIR
% 29.01.2015

% The function to view the multiple radars layout
% Inputs:
% ang: angular distribution type
% rng: geographical distribution type
% trg: 2x1 vector for the position of target in meters

% Outputs:
% M: the number of transmit radars
% N: the number of receive radars
% tx and ty: position of transmit radars in meters for x-y plane
% rx and ry: position of receive radars in meters for x-y plane
function [M,N,tx,ty,rx,ry]=viewlayout(ang,rng,trg)
THT = [5 85 150 265 330
       5 22  40  60  85
       5 22  40  60  80
      10 60 100 130 170];
tht = THT(ang,:);
RT = [3 3 3   3   3
      5 3 2.4 3.4 5]*1e+3;
rt = RT(rng,:);
M=numel(rt);
[tx,ty] = pol2cart(tht*pi/180,rt);
plot(tx,ty,'s','MarkerFaceColor','b')

THR = [0 50 100 175  205  260  310
       0 15  25  35   45   65   70
     -5 -10 -30 -35  -50  -60  -75
     -5 -35 -55 -85 -120 -150 -170];
thr = THR(ang,:);
RR = [3    3    3    3    3    3    3
      2.85 2.85 2.77 2.97 1.92 1.82 2.67]*1e+3;
rr = RR(rng,:);
N=numel(rr);
[rx,ry] = pol2cart(thr*pi/180,rr);
hold on
plot(rx,ry,'^','MarkerEdgeColor','g','MarkerFaceColor','g')
grid
strT =cell(1,M);
strR =cell(1,N);
for i=1:M
    strT{i} = ['T' num2str(i)];
end
for i=1:N
    strR{i} = ['R' num2str(i)];
end
grid
text(tx+100, ty+100,strT);

text(rx+100, ry+100,strR);
plot(trg(1),trg(2),'o','MarkerEdgeColor','k','MarkerFaceColor','k')
text(trg(1)+100, trg(2)+100,'Target');
grid
axlim = [-4 4 -4 4
         -1 4 -1 4
         -4 4 -4 4
         -4 4 -4 4
         -4 6 -4 6
         -1 6 -1 6
         -4 6 -4 6
         -6 6 -6 6]*1e+3;
axis(axlim(ang+(rng-1)*4,:))
axis square
title(['Case ' num2str(ang+(rng-1)*4)])
xlabel('x, m')
ylabel('y, m')
hold off