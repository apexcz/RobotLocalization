function [moveCommands turnCommands] = routePlan(botPos,currentAng,target,map)
    
    % find shortest path using the rapid random tree star =  rrt + A*
    moveCommands = [];
    turnCommands = [];
    
    stepSize = 5;
    num = 100;
    
    % set the co-ordinate,cost and parent of the start and node
    q_start.coord = botPos;
    q_start.cost = 0;
    q_start.parent = 0;
    q_goal.coord = target;
    q_goal.cost = 0;

    nodes = [];
    
    %particles used to get random pose within the map 
    planners(num,1) = BotSim; 
    
    wellPlanned=-1;
    itr = 0;
    
    while wellPlanned == -1
        itr = itr +1
        nodes = [];
        nodes =[nodes q_start];
        for i=1:num
            planners(i) = BotSim(map);
            planners(i).randomPose(0);
            %planners(i).drawBot(3);    

            % random coordinate
            q_rand = planners(i).getBotPos();

            % Break if target node is already reached
            for j = 1:1:length(nodes)
                if nodes(j).coord == q_goal.coord
                    break
                end
            end

            % Pick the closest node to the q_rand from existing list to branch out from
            ndist = [];
            for j = 1:1:length(nodes)
                tmp = trueDistance(j,nodes, q_rand,map);
                ndist = [ndist tmp];
            end
            [minDistance, idx] = min(ndist);
            q_near = nodes(idx);

            % get a coord in between the q_rand and q_near
            q_new.coord = navigate(q_rand, q_near.coord, minDistance, stepSize);

            %isnan(isCollision(...)) means no collision
            if isnan(isCollision(q_rand, q_near.coord,map))
                line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
                q_new.cost = distance(q_new.coord, q_near.coord) + q_near.cost; % evaluate the cost of the new node

                 % Within a radius of r, find all existing nodes
                q_nearest = [];
                r = 30;
                neighbor_count = 1;
                for j = 1:1:length(nodes)
                    check = isnan(isCollision(nodes(j).coord, q_new.coord,map));
                    if check & trueDistance(j,nodes, q_new.coord,map) <= r
                        q_nearest(neighbor_count).coord = nodes(j).coord;
                        q_nearest(neighbor_count).cost = nodes(j).cost;
                        neighbor_count = neighbor_count+1;
                    end
                end

                % Initialize cost to currently known value
                q_min = q_near;
                C_min = q_new.cost;

                % update the node that has the least cost
                for k = 1:1:length(q_nearest)
                    if isnan(isCollision(q_nearest(k).coord, q_new.coord,map)) & q_nearest(k).cost + trueDistance(k,q_nearest, q_new.coord,map) < C_min
                        q_min = q_nearest(k);
                        C_min = q_nearest(k).cost + trueDistance(k,q_nearest, q_new.coord,map);
                        %line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');                
                        hold on
                    end
                end

                % Update parent to least cost-from node
                for j = 1:1:length(nodes)
                    if nodes(j).coord == q_min.coord
                        q_new.parent = j;
                    end
                end

                % Append to nodes
                if planners(1).pointInsideMap([q_new.coord])
                    nodes = [nodes q_new];               
                end

            end
        end

        % get the distances of the nodes to the target
        localDistance = [];
        for j = 1:1:length(nodes)  
            tmpdist = trueDistance(j,nodes,q_goal.coord,map);
            localDistance = [localDistance tmpdist];
        end

        % Search backwards from goal to start to find the optimal least cost path
        [sortedDistance, distIdx] = sort(localDistance, 'ascend');

        % try to get 
        for j = 1:length(nodes)
            if planners(1).pointInsideMap(nodes(distIdx(:,j)).coord)
               indx =  distIdx(:,j);
               colli = isCollision(nodes(indx).coord,q_goal.coord,map);

               if isnan(colli)
                   wellPlanned = 1;
                   break
               end           
            end
        end    
    end
    
    % set the closest node to the target as the parent
    q_final = nodes(indx);
    q_goal.parent = indx;
    q_end = q_goal;

    incsteps =[];
    
    while q_end.parent ~= 0
        start = q_end.parent;
        line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'b', 'LineWidth', 2);
        hold on
        q_end = nodes(start);
        incsteps =[incsteps start];
    end

    %reverse the list since it is starting from the target to source,
    %instead of from source to target
    incsteps = fliplr(incsteps);
    
    for i= 1:length(incsteps)-1 % because it starts from the source which is a node
        coord = nodes(incsteps(i+1)).coord;
        rr = angle(botPos,coord);
        theta = mod(rr - currentAng,2*pi);
        gap = distance(botPos,coord);
        botPos = coord;
        currentAng =  theta + currentAng;
        
        moveCommands = [moveCommands gap];
        turnCommands = [turnCommands theta];
    end
    
    % finally link the closest node to the rank
    coord = q_goal.coord;
    rr = angle(botPos,coord);
    theta = mod(rr - currentAng,2*pi);
    gap = distance(botPos,coord);
    botPos = coord;

    moveCommands = [moveCommands gap];
    turnCommands = [turnCommands theta];
end