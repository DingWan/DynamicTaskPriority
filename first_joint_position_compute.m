function[joint_position_1]=first_joint_position_compute(q)
%%create 1-DoF mechanism
%the rotation of the second joint is no longer considered
syms q1 
%%4DoFs robot
q1=q(1);
L(1) = Link([pi/2 0 0 pi/2 1]);

q = [q1];

R1 = SerialLink(L,'name','R1');

homogeneous_transform_first_joint = R1.fkine(q);
joint_position_1=homogeneous_transform_first_joint(1:3,4);