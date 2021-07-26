%%compute the position of the forth joint of the planer 4-DoF mechanism.
function[joint_position_6]=wrist_joint_position_compute(q)
%%create -DoF mechanism
%the rotation of the forth joint is no longer considered
syms q1 q2 q3 q4 q5 q6 q7 q8 q9 q10  real
deg = pi/180;
%%4DoFs robot
q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);q7=q(7);q8=q(8);q9=q(9);q10=q(10);
L(1) = Link([pi/2 0 0 pi/2 1]); 
L(2) = Link([pi/2 0 0 pi/2 1]);
L(3) = Link([pi/2  0  0 pi/2 1]);

%% Create arm
%ball_joint
L(4)= Revolute('d', 3.15, 'a', 0, 'alpha', -pi/2 );
L(5) = Revolute('d', 0, 'a', 0, 'alpha', pi/2);
L(6) =  Revolute('d', 4.5, 'a', 0, 'alpha', -pi/2 );
L(7) =  Revolute('d', 0, 'a', 0, 'alpha', pi/2);
L(8) =  Revolute('d', 5, 'a', 0, 'alpha', -pi/2 );

q = [q1 q2 q3 q4 q5 q6 q7 q8 ];

R8 = SerialLink(L,'name','R8');
 n = R8.n;
 L = R8.links;
if numel(q) == n
    t = R8.base;
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
    i=5;
        t = t * [cos(q5),0,sin(q5),0;
                sin(q5),0,-cos(q5),0;
                0,1,0,0;
                0,0,0,1];
    i=6;
        t = t * [cos(q6),0,-sin(q6),0;
                sin(q6),0,cos(q6),0;
                0,-1,0,4.5;
                0,0,0,1]; 
    i=7;
        t=t*[cos(q7),0,sin(q7),0;
                sin(q7),0,-cos(q7),0;
                0,1,0,0;
                0,0,0,1];
    i=8;
        t=t*[cos(q8),0,-sin(q8),0;
                sin(q8),0,cos(q8),0;
                0,-1,0,5;
                0,0,0,1];
   
end
       t = t * R8.tool;

    if numcols(q) ~= n
        error('q must have %d columns', n)
    end
    t = zeros(4,4,0);
    for qv=q',		% for each trajectory point
        tt = R8.base;
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
            gg=qv(4);
            tt = tt * [cos(gg),0,sin(gg),5*cos(gg);
                sin(gg),0,-cos(gg),sin(gg)*5;
                0,1,0,0;
                0,0,0,1];
        i=5;
            hh=qv(5);
            tt = tt * [cos(hh),0,sin(hh),0.1*cos(hh);
                sin(hh),0,-cos(hh),sin(hh)*0.1;
                0,1,0,0;
                0,0,0,1];
        i=6;
            ii=qv(6);
            tt = tt * [cos(ii),0,sin(ii),0.1*cos(ii);
                sin(ii),0,-cos(ii),sin(ii)*0.1;
                0,1,0,0;
                0,0,0,1];
         i=7;
            jj=qv(7);
            tt = tt * [cos(jj),0,sin(jj),0;
                sin(jj),0,-cos(jj),0;
                0,1,0,0;
                0,0,0,1];   
         i=8;
            kk=qv(8);
            tt = tt * [cos(kk),0,-sin(kk),0;
                sin(kk),0,cos(kk),0;
                0,-1,0,5;
                0,0,0,1];  

        t = cat(3, t, tt * R8.tool);
        
    end
homogeneous_transform_wrist_joint = t;
joint_position_6=homogeneous_transform_wrist_joint(1:3,4);