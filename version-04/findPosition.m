function [priors, particles, curBot] = findPosition(priors, particles, curBot, botScan)
    num = length(priors);
    PosSigma = 2;
    AngSigma = 0.1;
    
    %% Write code for updating your particles scans
    for i = 1:num
        if particles(i).insideMap() == 0
            weights(i) = 0;
            priors(i) = 0;
        else
            particleScan = particles(i).ultraScan();
            weights(i) = calWeight(botScan, particleScan);
        end
    end
    
    %% Write code for scoring your particles
    % Uniformization: the sum of weight should equal to one.
    weights = (weights + priors) / 2;
    priors = weights;
    sumWeight = sum(weights);
    distributions = weights / sumWeight;
    
    % Predict the position of the robotic.
    for i = 1:num
        p = particles(i).getBotPos();
        positionX(i) = p(1);
        positionY(i) = p(2);
        positionA(i) = mod(particles(i).getBotAng(), 2 * pi);
    end
    BotPositionX = sum(distributions .* positionX);
    BotPositionY = sum(distributions .* positionY);
    tempSin = sum(distributions .* sin(positionA));
    tempCos = sum(distributions .* cos(positionA));
    BotPositionA = atan2(tempSin, tempCos);
    
    curBot.setBotPos([BotPositionX BotPositionY]);
    curBot.setBotAng(BotPositionA);

    %% Write code for resampling your particles
    TmpParticles = particles;
    [~, idxs] = histc(rand(num,1), [0 cumsum(distributions)]);
    for i = 1:num
        position = TmpParticles(idxs(i)).getBotPos() + normrnd(0,PosSigma,1,2);
        particles(i).setBotPos(position);
        particles(i).setBotAng(TmpParticles(idxs(i)).getBotAng() + normrnd(0,AngSigma));
        priors(i) = weights(idxs(i));
    end
    
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)
    for i = randi(num, [1, 200])
        particles(i).randomPose(0);
        priors(i) = 0;
    end
    
end