function [ theta ] = calAngle(A, B)
%% calAngle
% two points
% return the angle of the vector AB
theta = atan2(B(2) - A(2), B(1) - A(1));
end

