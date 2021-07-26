%global variable:limitation of joint angle range
global qdotlim
%% Create Symbolic Variables
syms q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 real
deg = pi/180;
%% Create redundant robot
L(1) = Link([pi/2 0 0 pi/2 1]); L(1).qlim = [0,30];
L(2) = Link([pi/2 0 0 pi/2 1]); L(2).qlim = [0,30];
L(3) = Link([pi/2 0 0 pi/2 1]);L(3).qlim = [0,30];
L(4)= Revolute('d', 3.15, 'a', 0, 'alpha', -pi/2 , ...
    'qlim', [-160 160]*deg);
L(5) = Revolute('d', 0, 'a', 0, 'alpha', pi/2, ...
    'qlim', [-160 160]*deg);
L(6) =  Revolute('d', 4.5, 'a', 0, 'alpha', -pi/2 , ...
    'qlim', [-160 160]*deg);
L(7) =  Revolute('d', 0, 'a', 0, 'alpha', pi/2, ...
    'qlim', [-160 160]*deg);
L(8) =  Revolute('d', 5, 'a', 0, 'alpha', -pi/2 , ...
    'qlim', [-160 160]*deg);
L(9) =  Revolute('d', 0, 'a', 0, 'alpha', pi/2 , ...
    'qlim', [-160 160]*deg);    
L(10) =  Revolute('d', 0.8, 'a', 0, 'alpha', 0, ...
    'qlim', [-160 160]*deg);

PR10 = SerialLink(L,'name','Protal3-PA10');

%% plot robot
q = [q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 ];
q_r = [5 10 5  0 -pi/3 pi/3 0 0 pi/4 0]; % ready pose, arm up
q_n = [10 5 10  0 -pi/4  pi/4 pi/3 -pi/3 0 pi/4];
q_s = [10 5 10  0 -pi/4  pi/4 4*pi/9 -pi/3 0 pi/4 ]; 
q_ie=[q_n;q_r];
%save
save('E:\\mini_thesis\\q_ie.mat','q_ie');