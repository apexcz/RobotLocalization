function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
scanNum = 6;
modifiedMap = shrinkMap(map, 5); %you need to do this modification yourself
botSim.setMap(map);
botSim.setScanConfig(botSim.generateScanConfig(scanNum));
curBot = BotSim(map);
curBot.setScanConfig(curBot.generateScanConfig(scanNum));

%generate some random particles inside the map
num =100; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(map);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    particles(i).setScanConfig(particles(i).generateScanConfig(scanNum));
end


%% Targets
% generate two random points to localise the robotic.
numTarget = 5;
curTarget = 1;
targets = zeros(numTarget, 2);
for i=1:numTarget-1
    targets(i,:) = curBot.getRndPtInMap(10);
end
targets(numTarget,:) = target;
target = targets(curTarget,:);


%% Short path
dijk = Dijkstra(modifiedMap, num, target);
dijk.init();


%% Localisation code
maxNumOfIterations = 10000;
n = 0;
converged = 0; %The filter has not converged yet
SIGMA = 2;
bias = 0.000001;


while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.

    
    %% Write code for updating your particles scans
    for i = 1:num
        if particles(i).insideMap() == 0
            weights(i) = bias;
        else
            particleScan = particles(i).ultraScan();
            weights(i) = 1 / (calWeight(botScan, particleScan) + bias);
        end
    end
    
    
    %% Write code for scoring your particles
    % Uniformization: the sum of weight should equal to one.
    sumWeight = sum(weights);
    for i = 1:num
        weights(i) = weights(i) / sumWeight;
    end
    
    % Predict the position of the robotic.
    for i = 1:num
        p = particles(i).getBotPos();
        positionX(i) = p(1);
        positionY(i) = p(2);
        positionA(i) = mod(particles(i).getBotAng(), 2 * pi);
    end
    BotPositionX = sum(weights .* positionX);
    BotPositionY = sum(weights .* positionY);
    BotPositionA = sum(weights .* positionA);
    BotPositionA = mod(BotPositionA, 2 * pi);
    
    curBot.setBotPos([BotPositionX BotPositionY]);
    curBot.setBotAng(BotPositionA);
    currentScan = curBot.ultraScan();
    currentWeight = 1 / (calWeight(botScan, currentScan) + bias);
	
    
    %% Write code for resampling your particles
    TmpParticles = particles;
    [~, idxs] = histc(rand(num,1), [0 cumsum(weights)]);
    for i = 1:num
        position = TmpParticles(idxs(i)).getBotPos() + normrnd(0,SIGMA,1,2);
        particles(i).setBotPos(position);
        particles(i).setBotAng(TmpParticles(idxs(i)).getBotAng() + normrnd(0,0.1));
    end
    
    
    %% Write code to check for convergence
    if and(currentWeight >= 1/scanNum, distance(curBot.getBotPos(), target) < 1)
        if curTarget == numTarget
            converged = 1;
            continue;
        else
            curTarget = curTarget + 1;
            target = targets(curTarget,:);
            dijk = Dijkstra(modifiedMap, num, target);
            dijk.init();
        end
    end
    
    
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)
    for i = randi(num, [1, 30])
        particles(i).randomPose(0);
    end
  
    
    %% Write code to decide how to move next
    next = dijk.find([BotPositionX, BotPositionY]);
    angle = calAngle([BotPositionX, BotPositionY], next);
    move = calMove(currentWeight, distance([BotPositionX, BotPositionY], next));
    turn = mod(angle - BotPositionA, 2*pi);
    
    
    %% Do the motation
    botSim.turn(turn); %turn the real robot.
    
    % avoid collision
    botScan = botSim.ultraScan();
    if botScan(1) < move
        botSim.turn(2 * pi - turn);
        continue;
    end
    
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end
    curBot.turn(turn);
    curBot.move(move);
   
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        for i =1:num
           particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        curBot.drawBot(10,'b');
        drawnow;
    end
    
    
end
end
