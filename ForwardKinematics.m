function [pos rot T01 T02 T03] = ForwardKinematics(q,wrist_offset)
%ForwardKinematics Performs forward kinematics
%   • Inputs (see Section A for a description of the following):
%       Length of links:
%           l_b
%           l_1
%           l_2
%           l_e
%       
%   • Outputs
%      Transformation Matricies

%Self note: [A] = DH(theta, d, a, alpha)

if nargin < 2
    wrist_offset = eye(4);
end


%     Reading the json file 
rob = roboStatCall;

% assigning the proper values
lb = rob.Links.Base.Length;
l1 = rob.Links.Link1.Length;
l2 = rob.Links.Link2.Length;
le = rob.Links.Connector.Length;

theta1 = q(1);
theta2 = q(2);
theta3 = q(3);

T01 = DH(0,0,lb,theta1);
T12 = DH(l1,0,0,(theta2+pi/2));
T23 = DH(l2,0,0,(theta3+pi/2));

T02 = T01*T12;
T03 = T01*T12*T23;

pos = T03(1:3,4);
rot = T03(1:3,1:3);
 

