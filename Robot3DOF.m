%clear
clc
clf
clear

%% Create Symbolic Variables
global qmax qmin Jacob Pend_velocity
syms q1 q2 q3

%%3DoFs robot
L(1) = Link([0 0 1 0]); L(1).qlim = [-2*pi/3,2*pi/3];
L(2) = Link([0 0 1 0]); L(2).qlim = [-2*pi/3,2*pi/3]; 
L(3) = Link([0 0 1.5 0]); L(3).qlim = [-2*pi/3,2*pi/3]; 

qdotlim = [-pi,pi;-pi,pi;-pi,pi];

q = [q1 q2 q3];
q_0= [-2*pi/3 0 -2*pi/3];
q_lb = [0,0,-2*pi/3];
q_ub = [0,0,1.9*pi/3];
q_1 = [pi/6,pi/6,pi/6];
q_2 = [pi/2,pi/2,pi/2];
%q_2 = [2*pi/6,2*pi/6,2*pi/6];

R3 = SerialLink(L,'name','R3');
%R3.plot(q_1);

%Forward kinematics
FTlb = R3.fkine(q_lb);
FTub = R3.fkine(q_ub); 
FT0 = R3.fkine(q_0);
FT1 = R3.fkine(q_1);
FT2 = R3.fkine(q_2);

%% Property index: H(q):joint limit
qmin = [L(1).qlim(:,1),L(2).qlim(:,1),L(3).qlim(:,1)];
qmax = [L(1).qlim(:,2),L(2).qlim(:,2),L(3).qlim(:,2)];
% 
% for i = 1:length(qmin)
% Hq(i) = (qmax(i)-qmin(i))^2/((qmax(i)-q(i))*(q(i)-qmin(i)));
% GradHq(i) = (qmax(i)-qmin(i))^2*(2*q(i)-qmin(i)-qmax(i))/((qmax(i)-q(i))*(q(i)-qmin(i)))^2;
% end

%% Cartesian Trajectory
T1 = transl(2, 0,0);
T2 = transl(-2, 1.5,0);
n = 51;

traj = ctraj(T1,T2,n);
for i = 1:n;
xtraj(i) = traj(1,4,i);
ytraj(i) = traj(2,4,i);
end
plot(xtraj,ytraj)

%% Velocity output of trajectory
locotime = 0:0.1:5;
[yt ytD ytDD] = lspb(2, -2, locotime); 
 % yt: y direction trajctory
[xt xtD xtDD] = lspb(1, 1, locotime);
 % xt: x direction trajctory
Pend_velocity = [xtD, ytD];

%% Inverse kinematics
% q = R3.ikine(traj(:,:,1),[],[1 1 0 0 0 1]);
% R3.plot(q);
factor = 1;
k = -0.001;
alpha = -0.05;
error = 1;
j = 0;

tic
while error
    j = j+1;
    k = -0.000001*j
for i = 1:n  
    
    if i == 1
        q = q_1;
        FTstart = R3.fkine(q);
        Pstart = transpose(FTstart(1:2,4));
        Pend =  transpose(traj(1:2,4,i));
        e = Pend - Pstart;      
    else
        Pstart =  transpose(traj(1:2,4,i-1));%, tr2rpy(traj(:,:,i))
        Pend =  transpose(traj(1:2,4,i));%, tr2rpy(traj(:,:,i+1))
        e = Pend - Pstart;        
    end
    
    Jacob = R3.jacob0(q,'trans');
    Jacob = Jacob(1:2,:);
    
    while norm(e)>0.001
        %% Gradient Projection Method
        %-------------- Gradient function of joint limit functionm Hq----------
        gradHq =  [((qmax(1) - qmin(1))^2*(2*q(1)- qmax(1)- qmin(1)))/((qmax(1) -q(1))^2*(q(1) - qmin(1))^2)
            ((qmax(2) - qmin(2))^2*(2*q(2)- qmax(2)- qmin(2)))/((qmax(2) -q(2))^2*(q(2) - qmin(2))^2)
            ((qmax(3) - qmin(3))^2*(2*q(3)- qmax(3)- qmin(3)))/((qmax(3) -q(3))^2*(q(3) - qmin(3))^2)];
        %----------------------------------------------------------------------
        
        %% Dynamic selection of factor k
%         if norm((eye(numel(q)) - pinv(Jacob) *Jacob ) * gradHq) ~= 0
%             k = alpha * norm (pinv(Jacob) * transpose(e))/ ...
%             (norm (pinv(Jacob) * transpose(e))+norm((eye(numel(q)) - pinv(Jacob) *Jacob ) * gradHq));
%         else
%             k =0;
%         end
        
        qhdot = k*(eye(numel(q)) - pinv(Jacob) *Jacob ) * gradHq;
        qsdot = pinv(Jacob) * transpose(e);
        delta_q =  qsdot + qhdot;
        q = q + factor * transpose(delta_q);
        Jacob = R3.jacob0(q,'trans');
        Jacob = Jacob(1:2,:);
        FTP = R3.fkine(q);
        P =  transpose(FTP(1:2,4));%, tr2rpy(FTP)]
        e = Pend - P;

    end 

    q_plot(i,:) = q;
    if q(1) > qmax(1) || q(2) > qmax(2) || q(3) > qmax(3) ||...
            q(1) < qmin(1) || q(2) < qmin(2) || q(3) < qmin(3) ...
            || det(Jacob*Jacob') == 0
        display('Exceed the boundary, Calculation process stopped');
        
        error = 1; break;        
    end
    
    if i == n
    error = 0;
    end
end
end
toc

R3.plot(q_plot);

%% Obtain required matrices and vectors for optimization
% M = numel(q);
% A = [eye(M);-eye(M);J(1:2,:)];
% Q = ones(1,M);
% b = [transpose(qmin);-transpose(qmax);transpose(e_velocity)];
% x0 = q; 
% val = [q1 q2 q3];
% s = 0.5; % scalar for Eq. 16
% objfunc = (qmax(1)-qmin(1))^2/((qmax(1) -q1)*(q1 -qmin(1))) + ...
%           (qmax(2)-qmin(2))^2/((qmax(2) -q2)*(q2 -qmin(2))) + ...
%           (qmax(3)-qmin(3))^2/((qmax(3) -q3)*(q3 -qmin(3)));
% %-------------- Gradient function of joint limit functionm Hq----------
% %         objfuncV = [((qmax(1) - qmin(1))^2*(2*q1-qmax(1)- qmin(1)))/(((qmax(1) -q1)*(q1 - qmin(1)))^2)
% %                     ((qmax(2) - qmin(2))^2*(2*q2-qmax(2)- qmin(2)))/(((qmax(2) -q2)*(q2 - qmin(2)))^2)
% %                     ((qmax(3) - qmin(3))^2*(2*q3-qmax(3)- qmin(3)))/(((qmax(3) -q3)*(q3 - qmin(3)))^2)];
%         %----------------------------------------------------------------------
%     %[x,minf] = minRosen(objfunc,A,b,x0,[q1 q2 q3],eps);
    

 %% Gradient Projection Method
%      x0 = q';
%      d = kfactor*(eye(numel(x)) - pinv(Jacob)*Jacob)*gradx;
%      q = q'
%     
%     R3.plot(q);   
    