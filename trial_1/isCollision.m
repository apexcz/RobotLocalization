function [ cp] = isCollision(linepoint1,linepoint2,map)
    %checks if the line between 2 points touches a wall boundary
    sz = size(map,1);
    cp = [NaN NaN];
    for i =1:sz
        mapLine=[];
        j = mod(i,sz) + 1;
        mapLine = [mapLine [map(i,1) map(i,2)] [map(j,1) map(j,2)]];
        crossingPoint = intersection([linepoint1 linepoint2],mapLine);
        
        dist = disToLineSeg(crossingPoint,[linepoint1 linepoint2]);
        
        if ~isnan(crossingPoint) & (dist == 0 || dist < 2) % 1.0e-10
            cp = crossingPoint;
            break;
        end        
    end
end