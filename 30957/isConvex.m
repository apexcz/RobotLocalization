function [ bo ] = isConvex(A, B, C)
% isConvex
AB = B - A;
AC = C - A;
angle = vectorProduct(AB, AC);
if angle < 0
    bo = 1;
else
    bo = 0;
end
end

