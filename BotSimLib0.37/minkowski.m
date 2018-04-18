function [ newMap ] = minkowski( radius, map, drawing )
%Inwardly offsets the map polygon to include the radius of the robot.
%
%	===== Inputs =====
%	radius 	- Size of offset in cm
%	map		- list of vertices of map polygon
%	drawing - outputs a graphical result

%	===== Outputs ====
%	newMap - List of vertices of new offset map
%

% intialising variables
numVert = size(map,1);
edges = zeros(numVert, 1);
angle = zeros(numVert, 1);
vec1 = zeros(numVert, 2);
vec2 = zeros(numVert, 2);
unitVec = zeros(numVert, 2);

% tests clockwise or anti-clockwise polygon
for i = 1:numVert
    point = map(i,:);
    if i == numVert
        point2 = map(1,:);
    else
        point2 = map(i+1,:);
    end
    edges(i) = (point2(1) - point(1))*(point2(2) + point(2));
end
direction = sum(edges); % -ve = anti-clockwise, +ve = clockwise

% calculates angles between adjacent edges of the map
for ii = 1:numVert
    current = map(ii,:);
    if ii == 1
        prev = map(numVert, :);
    else
        prev = map(ii-1,:);
    end
    if ii == numVert
        next = map(1, :);
    else
        next = map(ii+1, :);
    end

    vec1(ii,:) = next - current;
    vec2(ii,:) = prev - current;

    unitVec(ii,:) = vec1(ii,:)/norm(vec1(ii,:));
    x1 = vec1(ii,1);
    y1 = vec1(ii,2);
    x2 = vec2(ii,1);
    y2 = vec2(ii,2);
    angle(ii) = mod(atan2(x1*y2-x2*y1,x1*x2+y1*y2),2*pi)*180/pi;
end

% intialising variables
newAngle = angle./2;
newVec = zeros(numVert, 2);
newMap = zeros(numVert, 2);
offset = zeros(numVert, 1);

% sets direction of vector based on polygon direction
if direction > 0
    distance = -1;
elseif direction < 0
    distance = 1;
end

% Calculates vector length away from original vertex
for j = 1:numVert
    if angle(j) <= 180
        offset(j) = distance*radius/sind(newAngle(j));
    elseif angle(j) > 180 && angle(j) <= 270
        offset(j) = -distance*radius/sind(newAngle(j)-180);
    elseif angle(j) > 270
        offset(j) = -distance*radius/sind(newAngle(j)-180);
    end
end

% Adds new map point for each original map vertex
for jj = 1:numVert
    ang = newAngle(jj);
    rotation = [cosd(ang), sind(ang); -sind(ang), cosd(ang)];
    newVec(jj,:) = unitVec(jj,:) * rotation * offset(jj);
    newMap(jj,:) = map(jj,:) + newVec(jj,:);
end

if drawing == true
    % draws the original map
    axis equal
    hold on
    mapPlot = newMap;
    mapPlot(numVert+1,:) = newMap(1,:);
    % draws new map
    plot(mapPlot(:,1),mapPlot(:,2),'lineWidth',2,'Color','g');
end

end
