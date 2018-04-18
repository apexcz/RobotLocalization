function [ distanceOut] = trueDistance(sourceIndex,nodes,goalCoord,map)
    % gets the actual distance between 2 nodes with collisions in
    % consideration
    distanceOut = distance(nodes(sourceIndex).coord, goalCoord);
    if ~isnan(isCollision(nodes(sourceIndex).coord, goalCoord,map))
        % look for the child of the node
        for k = 1:1:length(nodes)
            if nodes(k).parent == sourceIndex && nodes(k).parent ~= 0
                distanceOut = distance(nodes(k).coord, nodes(sourceIndex).coord) + trueDistance(k,nodes, goalCoord,map);
                break
            end
        end       
    end    
end