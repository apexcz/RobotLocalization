function [ dist ] = calDisInMap( map, from, to )
% calculate the distance between two point in the map
number = length(map);
map(number+1,:) = map(1,:);

for i=1:number
    [right, left] = extendSeg(map(i,:), map(i+1,:), 3);
    if isIntersected(right, left, from, to) == 1
        dist = -1;
        return
    end
end

dist = distance(from , to);
end

