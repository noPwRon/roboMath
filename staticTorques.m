function tau = staticTorques(J,F,M)
% This function determines the torque required for each joint in the system
% 
% Inputs 
% J - The 6x3 jacobian matrix
% F - a 3x1 force vector
% M - a 3x2 moment vector
% 
% Ouputs
% tau - A 3x1 torque vector

if nargin < 3
    M = [0;0;0];
end

Jv = J(1:3,:);
Jw = J(4:6,:);

tau = -Jv'*F;


