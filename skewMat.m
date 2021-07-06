function A = skewMat(q)
% Inputs
%  q - a 3x1 matrix
% 
% Outputs 
%  A - The skew matrix equivulent

A = [0 q(3) -q(2);
    -q(3) 0 q(1);
    q(2) -q(1) 0];