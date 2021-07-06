function [J] = jacobianGen(T01, T02, T03)
% This function generates a new symbolic jacobian matrix in case the overall geometry experiences a major overhaul
%  
% Inputs
%   T01 - transformation matrix from frame 1 to frame 0
%   T02 - transformation matrix from frame 2 to frame 0
%   T03 - transformation matrix from frame 3 to frame 0  

o_0 = [0 0 0]';
o_1 = T01(1:3,4);
o_2 = T02(1:3,4);
o_3 = T03(1:3,4);

z_0 = [0 0 1]';
z_1 = T01(1:3,3);
z_2 = T02(1:3,3);
z_3 = T03(1:3,3);

J1 = [cross(z_0,(o_3-o_0)) ; z_0];
J2 = [cross(z_1,(o_3-o_1)) ; z_1];
J3 = [cross(z_2,(o_3-o_2)) ; z_2];

J = [J1,J2,J3];

end