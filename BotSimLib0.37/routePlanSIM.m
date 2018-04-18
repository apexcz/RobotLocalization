function [ botSim, moves] = routePlanSIM( botSim, mapGrid, limsMin, start, target, resolution, drawing )
%Plans a discretised route from the start to the target in steps using a wavefront path finder
%
%	===== Inputs =====
%	botSim 		- BotSim class
%	mapGrid		- Array of discretised map
%	limsMin 	- smallest corner of the map
%	start		- [x, y] coordinates of the start point
%	target 		- [x, y] coordinates of the target
%	resolution 	- size of route steps
%	drawing		- outputs a graphical result
%	===== Outputs ====
%	botSim 	- BotSim class
%	moves 	- List of unit moves from start to target
%	


numCells = sum(sum(mapGrid)); % Used in preallocation
mapSizeX = size(mapGrid, 2);
mapSizeY = size(mapGrid, 1);

% Real world coordinates in cm
startX = start(1);
startY = start(2);
targetX = target(1);
targetY = target(2);

% Grid world coordinates
startX = floor(startX/resolution) + (1-limsMin(1));
startY = mapSizeY - mod(floor(startY/resolution) + mapSizeY, mapSizeY);
targetX = floor(targetX/resolution) + (1-limsMin(1));
targetY = mapSizeY - mod(floor(targetY/resolution) + mapSizeY, mapSizeY);

cost = 1;           % 90 deg moves have a cost of 1
costDia = 1.414;    % 45 deg moves have a cost of sqrt(1^2 + 1^2)

delta = [-1, 0; %up
    -1, -1;%upleft
    0, -1; %left
    1, -1; %downleft
    1, 0;  %down
    1, 1;  %downright
    0, 1;  %right
    -1, 1];%upright

closed = zeros(mapSizeY, mapSizeX); % intialises the grid cells
closed(startY, startX) = 1;
action = ones(mapSizeY, mapSizeX).*-1;

g = 0;
open = [g, startY, startX]; % initialises start grid point and cost

found = false;
resign = false;

while found == false && resign == false
    if size(open, 1) == 0 % checks to see if there are any open moves left
        resign = true;
        disp('No path found');
    else
        [~, d2] = sort(open(:,1)); 	% sorts open moves by cost
        open = open(d2, :);			% replaces them in the array in order

        next = open(1,:);	% chooses the lowest cost moves
        open(1,:) = [];		% removes the chosen move

        g = next(1);
        y = next(2);
        x = next(3);

        if x == targetX && y == targetY
            found = true; 	% checks to see if the target has been found
        else
            % Stops the route cutting across corners by not allowing diagonals if by map edge
            %if y+1 <= mapSizeY && y-1 > 0 && x+1 <= mapSizeX && x-1 > 0
                %if mapGrid(y+1,x) == 0 || mapGrid(y-1,x) == 0 || mapGrid(y,x+1) == 0 || mapGrid(y, x-1) == 0
                    step = 2; % skips diagonal moves
                %else
                    %step = 1; % includes diagonal moves
                %end
            %end

            for i = 1:step:size(delta, 1) 	% iterates through each of the directions
                x2 = x + delta(i, 2);		% adds x coordinate of move
                y2 = y + delta(i, 1);		% adds y coordinate of move
                if x2 > 0 && x2 <= mapSizeX && y2 > 0 && y2<= mapSizeY 	% checks to see if the new grid cell is on map
                    if closed(y2, x2) == 0 && mapGrid(y2, x2) == 1;		% checks to see if it has not been visited and not an obstacle
                        %if delta(i, 1) == 0 || delta(i, 2) == 0			% diagonal move check
                            g2 = g + cost;			% adds cost of normal move
                        %else
                            %g2 = g + costDia;		% adds cost of diagnoal move
                        %end
                        n = size(open, 1)+1;		% determines size of 'open'
                        open(n, :) = [g2, y2, x2];	% adds new open grid cell
                        closed(y2, x2) = 1;			% closes old grid cell
                        action(y2, x2) = i;			% adds the direciton used
                    end
                end
            end
        end
    end
end

x = targetX;
y = targetY;
moves = zeros(numCells, 1); % Preallocation for speed
path = zeros(numCells, 2);	% Preallocation for speed

if resign == false
    n = 1;
	% Works backwards from target to start position to generate a list of moves
    while x ~= startX || y ~= startY
        x2 = x - delta(action(y, x), 2);	% uses direction to work backwards for x
        y2 = y - delta(action(y, x), 1);	% uses direction to work backwards for y
        moves(n) = action(y, x); 			% adds direction to 'moves'

        path(n,1) = (x2-1) * resolution + limsMin(1);						% generates 'real world' path
        path(n,2) = (mapSizeY - mod(y2+mapSizeY, mapSizeY)) * resolution; 	% generates 'real world' path

        x = x2;		% sets current x to be used to calculate next x
        y = y2;		% sets current y to be used to calculate next y
        n = n + 1; 	% steps counter
    end
end

moves = moves(moves > 0);  	% Removes zero elements from preallocation
path = path(path > 0);		% Removes zero elements from preallocation

if found == true
    if size(moves, 1) ~= 0
        moves = flip(moves); % Flips array to be in correct order
    end
else
    moves = 0; 		% if no moves found set to 0
end
end
