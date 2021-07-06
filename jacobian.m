function [J] = jacobian(q, T01,T02,T03)
% Generates a jacobian matrix based on the current robotic model and joint angles q
% 
% Inputs
%   q - a 3x1 matrix of joint angles
%   T01 - transformation matrix from frame 1 to frame 0
%   T02 - transformation matrix from frame 2 to frame 0
%   T03 - transformation matrix from frame 3 to frame 0 
%  
% Outputs
%   Jacobian matrix

%Self note: [A] = DH(theta, d, a, alpha)

rob = roboStatCall;

lb = rob.Links.Base.Length;
l1 = rob.Links.Link1.Length;
l2 = rob.Links.Link2.Length;
le = rob.Links.Connector.Length;

theta1 = q(1);
theta2 = q(2);
theta3 = q(3);


o_0 = [0;0;0];
o_1 = T01(1:3,4);
o_2 = T02(1:3,4);
o_3 = T03(1:3,4);

o = [o_0 o_1 o_2 o_3];

z_0 = [0;0;1];
z_1 = T01(1:3,3);
z_2 = T02(1:3,3);
z_3 = T03(1:3,3);

z = [z_0 z_1 z_2 z_3];

for k=1:length(o)-1
    J(1:6,k) = [cross(z(1:3,k),(o(1:3,end)-o(1:3,k)));z(1:3,k)];
end

Jv = J(1:3,:);
Jw = J(4:6,:);

end