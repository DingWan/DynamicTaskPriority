%%compute the position of the forth joint of the planer 4-DoF mechanism.
function[joint_position_3]=third_joint_position_compute(q)
%%create -DoF mechanism
%the rotation of the forth joint is no longer considered
syms q1 q2 q3 
%%4DoFs robot
q1=q(1);q2=q(2);q3=q(3);
L(1) = Link([pi/2 0 0 pi/2 1]);
L(2) = Link([pi/2 0 0 pi/2 1]); 
L(3) = Link([pi/2  0  0 pi/2 1]);

q = [q1 q2 q3 ];

R3 = SerialLink(L,'name','R3');
n = R3.n;
L = R3.links;
if numel(q) == n
    t = R3.base;
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
end
       t = t * R3.tool;

    if numcols(q) ~= n
        error('q must have %d columns', n)
    end
    t = zeros(4,4,0);
    for qv=q',		% for each trajectory point
        tt = R3.base;
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
        t = cat(3, t, tt * R3.tool);
        
    end
homogeneous_transform_third_joint = t;
joint_position_3=homogeneous_transform_third_joint(1:3,4);