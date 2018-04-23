function [ shrink ] = shrinkMap(originMap, d)
% shrinkMap
number = length(originMap);
shrink = zeros(number, 2);

originMap(number+1,:) = originMap(1,:);
originMap(number+2,:) = originMap(2,:);

cntO = 0;
cntZ = 0;

for i=1:number
    if isConvex(originMap(i,:), originMap(i+1,:), originMap(i+2,:)) == 1
        cntO = cntO + 1;
    else
        cntZ = cntZ + 1;
    end
end

if cntO > cntZ
    flag = 1;
else
    flag = 0;
end

for i=1:number
    u = originMap(i,:) - originMap(i+1,:);
    u = u / norm(u);
    
    v = originMap(i+2,:) - originMap(i+1,:);
    v = v / norm(v);
    
    w = u + v;
    w = w / norm(w);
    
    if isConvex(originMap(i,:), originMap(i+1,:), originMap(i+2,:)) == flag
        shrink(i,:) = originMap(i+1,:) + (w * d);
    else
        shrink(i,:) = originMap(i+1,:) - (w * d);
    end
end

end

