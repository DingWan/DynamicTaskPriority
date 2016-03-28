clc
clf
clear

%% Create Symbolic Variables  5DoFs robot
L(1) = Link([0 0     0  pi/2  1]); L(1).qlim = [1,1];
L(2) = Link([0 0.315 0  -pi/2]); L(2).qlim = [-177*pi/180,177*pi/180];
L(3) = Link([0 0     0  pi/2]); L(3).qlim = [-91*pi/180,91*pi/180];
L(4) = Link([0 0.450 0  -pi/2]); L(4).qlim = [-174*pi/180,174*pi/180];
L(5) = Link([0 0 0.500  0]); L(5).qlim = [-137*pi/180,137*pi/180];

q0 = [0 pi/6 pi/6 0 -105*pi/108];
q1 = [0.5,0,0,0];



R5 = SerialLink(L,'name','R5');
R5.plot(q0,'workspace',[-2,2,-2,2,-2,2]);
%R5.plot([0.5,0]);

qdotlim = [-pi,pi;-pi,pi;-pi,pi];

q = [q1 q2 q3 q4 q5];
q_0= [0 pi/6 pi/6 0 105*pi/108];
q_lb = [0,0,-2*pi/3];
q_ub = [0,0,1.9*pi/3];
q_1 = [pi/6,pi/6,pi/6];
q_2 = [pi/2,pi/2,pi/2];

R5.plot(q_0);

%Forward kinematics
FTlb = R5.fkine(q_lb);
FTub = R5.fkine(q_ub); 
FT0 = R5.fkine(q_0);
FT1 = R5.fkine(q_1);
FT2 = R5.fkine(q_2);

%% Property index: H(q):joint limit
qmin = [L(1).qlim(:,1),L(2).qlim(:,1),L(3).qlim(:,1)];
qmax = [L(1).qlim(:,2),L(2).qlim(:,2),L(3).qlim(:,2)];

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