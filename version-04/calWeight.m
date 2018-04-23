function [ weight ] = calWeight(disA, disB)
%%  calWeight
% calculate the weight between two array
% SIG = 3; % 12 direction and threadhold is 3

% weight = 0;
% for i=1:length(disA)
%     weight = weight + gaussmf(disB(i), [SIG, disA(i)]);
% end

% SIG = 5; % 12 direction and threadhold is 3
% 
% weight = 1;
% for i=1:length(disA)
%     weight = weight * gaussmf(disB(i), [SIG, disA(i)]);
% end

% weight = gaussmf(weight, [SIG, 6]);

SIG = 500;
x = disA - disB;
weight = sum(x.*x);
weight = gaussmf(weight, [SIG, 0]);

end

