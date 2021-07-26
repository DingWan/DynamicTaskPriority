%%compute the position of the third joint of the planer 4-DoF mechanism.
function[joint_position_2]=second_joint_position_compute(q)
%%create 2-DoF mechanism
%the rotation of the third joint is no longer considered
syms q1 q2 
%%4DoFs robot
q1=q(1);q2=q(2);
L(1) = Link([pi/2 0 0 pi/2 1]);
L(2) = Link([pi/2 0 0 pi/2 1]); 


q = [q1 q2 ];

R2 = SerialLink(L,'name','R2');
%% 
L =R2.links;
n = R2.n;
if numel(q) == n
    t = R2.base;
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
end
       t = t * R2.tool;

    if numcols(q) ~= n
        error('q must have %d columns', n)
    end
    t = zeros(4,4,0);
    for qv=q',		% for each trajectory point
        tt = R2.base;
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
        t = cat(3, t, tt * R2.tool);
        
    end
       t;

homogeneous_transform_second_joint = t;
joint_position_2=homogeneous_transform_second_joint(1:3,4);