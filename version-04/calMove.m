function [ move ] = calMove(weight, dist)
% decide the length of each step depending weight
% if weight > 1/6
%     move = dist;
% else
%     if weight > 1/100
%         move = 0.5 * dist;
%     else
%         move = 1;
%     end
% end
% 
% if move > 5
%     move = 5;
% end
move=5
end

