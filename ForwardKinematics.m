function [pos rot T01 T02 T03 T04] = ForwardKinematics(q,wrist_offset)
% Determines the position and rotation of the end effector of the marine robot
% 
% Inputs 
%   q - a 3x1 vector of joint angles
%   wrist_offset - The transoformation matrix from the wrist to end effector tip (Optional)
% 
% Outputs
%   pos - position of the end effector
%   rot - rotation matrix of the end effector
%   T01 - transformation matrix from frame 1 to frame 0
%   T02 - transformation matrix from frame 2 to frame 0
%   T03 - transformation matrix from frame 3 to frame 0  
    

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

T01 = DH(0,pi/2,lb,theta1);
T12 = DH(l1,0,0,(theta2+pi/2));
T23 = DH(l2,0,0,(theta3+pi/2));

T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T03*wrist_offset;

pos = T04(1:3,4);
rot = T04(1:3,1:3);
 

