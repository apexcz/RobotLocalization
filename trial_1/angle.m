function [ angleOut ] = angle( source,target )
%angle returns the angle between two points
x1 = source(:,1);
y1= source(:,2);
x2 = target(:,1);
y2 = target(:,2);

global theta;

theta = 0;

if x1 ~= x2 || y1 ~= y2
    theta = atan2(y2 - y1,x2 - x1);
    
    if theta < 0
        theta = theta + (2 * pi);
    end
end

angleOut =  theta;
end