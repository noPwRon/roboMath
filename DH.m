function [A] = DH(a, alpha, d, theta)
%DH Denavit-Hartenberg (D-H) representation
%   • Inputs (see Section A for a description of the following):
%       – theta: Angle measured in radians
%       – d: Distance measured in meters
%       – a: Length measured in meters
%       – alpha: Twist measured in radians
%   • Outputs
%       - A: The homogeneous matrix
% 
% A = R(z,theta) * T(z,d) * T(x,a) * R(x,alpha)


A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
    0   sin(alpha)  cos(alpha)  d;
    0 0 0 1];
end