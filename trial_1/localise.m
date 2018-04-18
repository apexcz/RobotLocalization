function localise(map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

    scanPower = 100;
    scans = 20;
    
    %hasCrashed = 1;
    trials=0; % try 10 times
    %while hasCrashed == 1 && trials <10
    while trials <10
        trials = trials + 1;
        % get robot's pose
        [pose] = robotPose(map,target);        
        
        if isnan(pose) % couldn't determine pose, relocalise           
            continue;
        end
        botpo = pose(1:2);
        
        %hold off;
        %botSim.drawMap();
        
        % find shortest path without colliding
        [moves,turns] = routePlan(botpo,pose(3),target,map);

        for i=1:length(moves)
            %botSim.turn(turns(i));
            movement(turns(i),0)
            [proximity, ~] = ultraScan(scanPower,scans); %get a scan from the real robot.
            move = moves(i);
            
            movement(0,move);
            
            % move bot if distance to cover wont hit the wall plus padding(2) 
%             if proximity(1,:) > move + 2
%                 botSim.move(move);
%                 botSim.drawBot(3);
%                 hasCrashed = 0;
%             else
%                 hasCrashed = 1;
%                 break;
%             end            
        end
    end
end
