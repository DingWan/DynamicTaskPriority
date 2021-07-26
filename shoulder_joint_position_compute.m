%%compute the position of the forth joint of the planer 4-DoF mechanism.
function[joint_position_4]=shoulder_joint_position_compute(q)
%%create -DoF mechanism
%the rotation of the forth joint is no longer considered
syms q1 q2 q3 q4   real
deg = pi/180;
%%4DoFs robot
q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);
L(1) = Link([pi/2 0 0 pi/2 1]); 
L(2) = Link([pi/2 0 0 pi/2 1]);
L(3) = Link([pi/2 0 0 pi/2 1]);
%% Create arm
%ball_joint
L(4)= Revolute('d', 3.15, 'a', 5, 'alpha', pi/2);

q = [q1 q2 q3 q4 ];

R4 = SerialLink(L,'name','R6');

%% fkine
 n = R4.n;
 L = R4.links;
if numel(q) == n
    t = R4.base;
    i=1;
        t = t * [0,0,1,0;
                1,0,0,0;
                0,1,0,q1;
                0,0,0,1];
    i=2;
        t = t * [0,0,1,0;
                1,0,0,0;
                0,1,0,q2;
                0,0,0,1];
    i=3;
        t = t * [0,0,1,0;
                1,0,0,0;
                0,1,0,q3;
                0,0,0,1];
    i=4;
        t = t * [cos(q4),0,-sin(q4),0;
                sin(q4),0,cos(q4),0;
                0,-1,0,3.15;
                0,0,0,1];

end
       t = t * R4.tool;

    if numcols(q) ~= n
        error('q must have %d columns', n)
    end
    t = zeros(4,4,0);
    for qv=q',		% for each trajectory point
        tt = R4.base;
        i=1;
            aa=qv(1);
            tt = tt * [0,0,1,0;
                    1,0,0,0;
                    0,1,0,aa;
                    0,0,0,1];
        i=2;
            bb=qv(2);
            tt = tt * [0,0,1,0;
                    1,0,0,0;
                    0,1,0,bb;
                    0,0,0,1];
        i=3;
            cc=qv(3);
            tt = tt * [0,0,1,0;
                    1,0,0,0;
                    0,1,0,cc;
                    0,0,0,1];
         i=4;
            dd=qv(4);
            tt = tt * [cos(dd),0,-sin(dd),0;
                sin(dd),0,cos(dd),0;
                0,-1,0,3.15;
                0,0,0,1];       
 
        t = cat(3, t, tt * R4.tool);
        
    end
homogeneous_transform_shoulder_joint = t;
joint_position_4=homogeneous_transform_shoulder_joint(1:3,4);