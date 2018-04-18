function [ mapGrid, limsMin ] = inMap( map, mink, resolution, debug)
% Test to see what points are inside the map
%
%	===== Inputs =====
%	map			- list of vertices of map polygon
%	mink		- minkowski map
%	resolution 	- size of route steps
%	===== Outputs ====
%	mapGrid		- Array of discretised map
%	limsMin 	- smallest corner of the map
%	

mapOrig = map;

TEMPmap = mink;
TEMPbotSim = BotSim(TEMPmap);

limsMin = min(mapOrig); % minimum limits of the original map
limsMax = max(mapOrig); % maximum limits of the original map
dims = limsMax - limsMin; %dimension of the map
iterators = dims/resolution;
iterators = ceil(iterators) + [1 1];
mapArray = zeros(iterators);

%loops through the grid indexes and tests if they are inside the map
for i = 1:iterators(2)
    for j = 1:iterators(1)
        testPos = limsMin + [j-1 i-1]*resolution;
        mapArray(i, j) = TEMPbotSim.pointInsideMap(testPos);
            if debug == true
                hold on
                if mapArray(i,j)
                    plot(testPos(1),testPos(2),'om');%inside map
                else
                    plot(testPos(1),testPos(2),'sg');%outside map
                end
            end
    end
end

mapGrid = flip(mapArray, 1); % fixes inverted y axis

end
