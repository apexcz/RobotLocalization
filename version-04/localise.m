function [botReal, curBot] = localise(botReal, map, target)


%% Local Variables
scanNum = 8;

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
priors = zeros(1, num);
distributions = zeros(1, num);

initTime = clock;
limitTime = 120;

while(converged == 0 && n < maxNumOfIterations && etime(clock, initTime) < limitTime)  %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botReal.ultraScan(); %get a scan from the real robot.
    
    for k =1:5
        [priors, particles, curBot] = findPosition(priors, particles, curBot, botScan);
    end

    %% Write code to check for convergence
    currentScan = curBot.ultraScan();
    currentWeight = calWeight(botScan, currentScan);
    disp(currentWeight);
    if currentWeight > 0.9
       converged = 1;
       continue;
    end
    
    %% Write code to decide how to move next
%     [move, turn] = findDirection(botScan);
    move = 25;
    turn = 0;
    if botScan(1) < move + 5
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
    curBot.drawBot(10,'b');
    drawnow;
end

end
