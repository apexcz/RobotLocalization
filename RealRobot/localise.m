function [botReal, curBot] = localise(botReal, map, target)


%% Local Variables
scanNum = 8;
PosSigma = 5;
AngSigma = 0.2;
modifiedMap = shrinkMap(map, 15); %you need to do this modification yourself

botReal = botReal.setScan(scanNum);

% Predict Bobotic
curBot = BotSim(map);
curBot.setScanConfig(curBot.generateScanConfig(scanNum));


%% Particles generation
% generate some random particles inside the map
num =500; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(map);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    particles(i).setScanConfig(particles(i).generateScanConfig(scanNum));
end


%% Localisation initialisation
maxNumOfIterations = 100;
n = 0;
converged = 0; % The filter has not converged yet
priors = ones(1, num);

while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botReal.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    for i = 1:num
        if particles(i).insideMap() == 0
            weights(i) = 0;
        else
            particleScan = particles(i).ultraScan();
            weights(i) = calWeight(botScan, particleScan);
        end
    end 
    
    %% Write code for scoring your particles
    % Uniformization: the sum of weight should equal to one.
    sumWeight = sum(weights);
    weights = weights / sumWeight;
    
    % multiple the prior distribution -- TODO
    weights = weights .* priors;
    sumWeight = sum(weights);
    weights = weights / sumWeight;
    
    
    % Predict the position of the robotic.
    for i = 1:num
        p = particles(i).getBotPos();
        positionX(i) = p(1);
        positionY(i) = p(2);
        positionA(i) = mod(particles(i).getBotAng(), 2 * pi);
    end
    BotPositionX = sum(weights .* positionX);
    BotPositionY = sum(weights .* positionY);
    tempSin = sum(weights .* sin(positionA));
    tempCos = sum(weights .* cos(positionA));
    BotPositionA = atan2(tempSin, tempCos);
    
    curBot.setBotPos([BotPositionX BotPositionY]);
    curBot.setBotAng(BotPositionA);

	
    %% Write code to check for convergence
    currentScan = curBot.ultraScan();
    currentWeight = calWeight(botScan, currentScan);
    disp(currentWeight);
    if currentWeight > 0.8
       converged = 1;
       continue;
    end
    
    %% Write code for resampling your particles
    TmpParticles = particles;
    [~, idxs] = histc(rand(num,1), [0 cumsum(weights)]);
    for i = 1:num
        position = TmpParticles(idxs(i)).getBotPos() + normrnd(0,PosSigma,1,2);
        particles(i).setBotPos(position);
        particles(i).setBotAng(TmpParticles(idxs(i)).getBotAng() + normrnd(0,AngSigma));
        priors(i) = weights(idxs(i));
    end
    
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)
    for i = randi(num, [1, 100])
        particles(i).randomPose(0);
        priors(i) = 0.3;
    end

    %% Write code to decide how to move next
    move = 10;
    turn = 0;
    if or(botScan(1) < 13, or(botScan(2) < 15, botScan(scanNum) < 15))
        move = 0;
        if mod(n, 8) >= 4
            turn = pi/2;
        else
            turn = -pi/2;
        end
    end
    
    
    %% Do the motation
    botReal.turn(turn);
    botReal.move(move);
    for i =1:num
        particles(i).turn(turn);
        particles(i).move(move);
    end
    curBot.turn(turn);
    curBot.move(move);

    
    %% Drawing
    hold off; %the drawMap() function will clear the drawing when hold is off
    curBot.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
    for i =1:num
        particles(i).drawBot(3); %draw particle with line length 3 and default color
    end
%     botReal.drawBot(30,'r');
    curBot.drawBot(10,'b');
    drawnow;
    
end

%% Short Path
maxNumOfIterations = 100;
n = 0;
converged = 0; % The filter has not converged yet

dijk = Dijkstra(modifiedMap, 100, target);
dijk.init();

while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    Position = curBot.getBotPos();
    Angle = curBot.getBotAng();
    
    if distance(Position, target) < 1
        converged = 1;
        continue;
    end
    
    next = dijk.find(Position);
    angle = calAngle(Position, next);
    move = distance(Position, next);
    turn = mod(angle - Angle, 2*pi);
    
    botReal.turn(turn);
    botReal.move(move);
    curBot.turn(turn);
    curBot.move(move);
    
    %% Drawing
    hold off; %the drawMap() function will clear the drawing when hold is off
    curBot.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
%     botReal.drawBot(10,'r');
    curBot.drawBot(10,'b');
    drawnow;
end

end
