function [ weight ] = calWeight(disA, disB)
%%  calWeight
% calculate the weight between two array
x = disA - disB;
weight = sum(x.*x);
end

