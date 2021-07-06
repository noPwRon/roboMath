function val = roboStatCall
% This function loads the json file containing the characteristic values of the marine charging robot
% 
% Outputs
% val - a struct containing the robot characteristic values

fname = 'RoboStats.json'; 
fid = fopen(fname); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
val = jsondecode(str);

