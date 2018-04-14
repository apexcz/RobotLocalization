classdef Dijkstra < handle
    %% Dijkstra
    
    properties (SetAccess = private,GetAccess = private)
        map;        % map
        tmpBot;     % for generate points
        number;     % the num of points
        points;     % points
        dists;
        graph;
    end
    
    methods
        function dijk = Dijkstra(map, number, target)
            dijk.map = map;
            dijk.tmpBot = BotSim(map);
            dijk.number = number;
            dijk.dists = zeros(1, number);
            
            dijk.points = zeros(number, 2);
            dijk.dists = zeros(number, 1);
            dijk.points(1,:) = target;
        end
        
        function init(dijk)
            for i = 2:dijk.number
                dijk.points(i,:) = dijk.tmpBot.getRndPtInMap(5);
                dijk.dists(i) = -1;
            end
            
            dijk.graph = zeros(dijk.number, dijk.number);
            for i = 1:dijk.number
                for j = 1:dijk.number
                    dijk.graph(i,j) = calDisInMap(dijk.map, dijk.points(i,:), dijk.points(j,:));
                end
            end
                        
            queue = Queue();
            queue.add(1);
            
            while queue.empty() == 0
                s = queue.pop();
                for i=1:dijk.number
                    if or(dijk.dists(s) == -1, dijk.graph(i,s) == -1)
                        continue;
                    end
                    if or(dijk.dists(i) == -1, dijk.dists(i) > dijk.dists(s) + dijk.graph(i,s))
                        dijk.dists(i) = dijk.dists(s) + dijk.graph(i,s);
                        queue.add(i);
                    end
                end
            end

        end 
        
        function pos = find(dijk, start)
            dis = -1;
            idx = 0;
            while idx == 0
                for i=1:dijk.number
                    tmp = calDisInMap(dijk.map, dijk.points(i,:), start);

                    if or(dijk.dists(i) == -1, tmp == -1)
                        continue;
                    end
                    if and(i ~= 1, tmp < 5)
                        continue
                    end
                    if or(dis == -1, dis > dijk.dists(i) + tmp)
                        dis = dijk.dists(i) + tmp;
                        idx = i;
                    end
                end
                
                if idx == 0
                    dijk.tmpBot.setBotPos(start);
                    if dijk.tmpBot.insideMap() == 0
                        idx = 1;
                    else
                        dijk.init();
                    end
                end
            end
            
            pos = dijk.points(idx,:);
        end
    end
    
end
