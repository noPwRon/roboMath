% Marine Charger
% 
%  DH Table Parameters
%    __________________________________________
%   |  a_i  |  alpha_i  |  d_i  |    theta_i   |  
% 1 |   0   |     0     |  l_b  |    theta_1   |
% 2 |   0   |    pi/2   |   0   |    theta_2   |
% 3 | -l_1  |     0     |   0   | theta_3+pi/2 |
% 4 | -l_2  |     0     |   0   | theta_4-pi/2 |
% 5 | -l_e  |   -pi/2   |   0   |    theta_5   |
%   -----------------------------------------


clc
clear

rob = roboStatCall;

l_b = rob.Links.Base.Length;
l_1 = rob.Links.Link1.Length;
l_2 = rob.Links.Link2.Length;
l_e = rob.Links.Connector.Length;


% l_b l_1 l_2 l_e 

syms c1 c2 c3 s1 s2 s3 lb l1 l2 theta1 theta2 theta3



T01 = [c1 -s1 0 0;
    s1 c1 0 0;
    0 0 1 lb;
    0 0 0 1];
T12 = [-s2 -c2 0 -l1*s2;
    c2 -s2 0 l1*c2;
    0 0 1 0;
    0 0 0 1];
T23 = [-s3 -c3 0 -l2*s2;
    c3 -s3 0 l2*c2;
    0 0 1 0;
    0 0 0 1];

T02 = T01*T12;
T03 = T01*T12*T23;

pos = T03(1:3,4);

[pos rot T01 T02 T03] = ForwardKinematics([0,0,0]);
q1 = inverseKinematics(pos);
q2 = inverseKinematicsSpong(pos);


J = jacobian([0,0,0],T01,T02,T03);
Jv = J(1:3,:);
Jw = J(4:6,:);

F = [0;100*9.81;0];

tau = staticTorques(J,F)


