function [J] = jacobian(q, T01,T02,T03)
%ForwardKinematics Performs forward kinematics
%   • Inputs (see Section A for a description of the following):
%       Length of links:
%           l_b
%           l_1
%           l_2
%           l_e
%       Joint Angles:
%           theta_1
%           theta_2
%           theta_3
%           theta_4
%           theta_5
%       
%   • Outputs
%      Jacobian matrix

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