function [q] = inverseKinematicsSpong(p)
    % Determines the required joint angles for a given coordinate in [x,y,z]
    % 
    % Inputs
    % x - x location of the wrist
    % y - y location of the wrist
    % z - z location of the wrist
    % 
    % Outputs
    % theta1 - joint 1 in radians
    % theta2 - joint 2 in radians
    % theta3 - joint 3 in radians
    
%     Reading the json file 
rob = roboStatCall;

% assigning the proper values
lb = rob.Links.Base.Length;
l1 = rob.Links.Link1.Length;
l2 = rob.Links.Link2.Length;
le = rob.Links.Connector.Length;
    
% labelling x y and z
x= p(1);
y= p(2);
z= p(3);
d=0
% magnitude of the vector to the connector
r = x^2+y^2+z^2;
s = z;
D = (r-l1^2-l2^2)/(2*l1*l2);


% determing angles
theta1 = atan2(y/sqrt(x^2+y^2),x/sqrt(x^2+y^2));
theta3 = atan2(sqrt(1-D^2),D);
theta2 = atan2(s,r) - atan2(l2*sin(theta3),l1+l2*cos(theta3));

q = [theta1 theta2 theta3];