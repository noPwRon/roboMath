function [J] = jacobianGen(T01, T02, T03)
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


o_0 = [0 0 0]';
o_1 = T01(1:3,4);
o_2 = T02(1:3,4);
o_3 = T03(1:3,4);
% o_4 = T04(1:3,4);
% o_5 = T05(1:3,4);

z_0 = [0 0 1]';
z_1 = T01(1:3,3);
z_2 = T02(1:3,3);
z_3 = T03(1:3,3);
% z_4 = T04(1:3,3);
% z_5 = T05(1:3,3);

J1 = [cross(z_0,(o_3-o_0)) ; z_0];
J2 = [cross(z_1,(o_3-o_1)) ; z_1];
J3 = [cross(z_2,(o_3-o_2)) ; z_2];
% J4 = [cross(z_3,(o_5-o_3)) ; z_3];
% J5 = [cross(z_4,(o_5-o_4)) ; z_4];



% J = [J1,J2,J3,J4,J5];
J = [J1,J2,J3];

end