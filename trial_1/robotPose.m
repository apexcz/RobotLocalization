function [guess] = robotPose(map,target)
%This function returns botSim and a guess, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
%botSim.setMap(modifiedMap);

scanPower = 100;
        
guess = [NaN NaN NaN];

%generate some random particles inside the map
num =300; % number of particles

scanNoise = 1; 
moveNoise = 0.01; 
turnNoise = 0.005;
hitRange = 3; 
scans= 20; 

%botSim.setScanConfig(botSim.generateScanConfig(scans));

path_div = 3;
motionStep = 1;
scan_max_ind = NaN;
angle_interval_bot = (0:(360/scans):360-(360/scans))...
    .* (pi()/180); %create array of possible ultrasound angles


particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).setMotionNoise(moveNoise);
    particles(i).setTurningNoise(turnNoise);
    particles(i).setSensorNoise(scanNoise);
    particles(i).randomPose(hitRange); %spawn the particles in random locations
    particles(i).setScanConfig(particles(i).generateScanConfig(scans));   
end

pose = zeros(num,3);

%% Localisation code
maxNumOfIterations = 50;
n = 0;
converged =0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    %botScan = botSim.ultraScan(); 
    [botScan, ~] = ultraScan(scanPower,scans); %get a scan from the real robot.
    
    weights = zeros(num,1);
   
   for i = 1 : num
       %% Write code for updating your particles scans
        particleScan = particles(i).ultraScan; %particle scan
        
        %% Write code for scoring your particles  
        similarity = zeros(scans,1);
        for j = 1 : scans
            similarity(j,:) = 1/norm(particleScan - botScan); % similarity factor = sqrt(sse(A-B))
            particleScan = circshift(particleScan,-1); %repeat at every angle
            if ~isfinite(similarity(j,:))
                similarity(j,:) = 1.5 % assign a scale of 1.5 for a perfect match
            end
        end
        [weights(i,:),maxOrientationIndex] = max(similarity);
        bestAngle = (maxOrientationIndex-1)*(pi/scans) * 2;
        particles(i).turn(bestAngle);   % turn particle to the estimated direction
    end
    weights = weights/sum(weights); % weight normalized to sum 1
    
    %% Write code for resampling your particles
    [maxWeight,maxIndex] = max(weights);
    beta=0;
    index=randi(num);
    for i = 1 : num
        beta = beta + rand(1)*2*maxWeight;
        %Replace only particles with weight smaller than beta
        while beta > weights(index) 
            beta = beta - weights(index);
            index = mod(index,num)+1; 
            weights(i) = weights(index);
            particles(i).setBotPos(particles(index).getBotPos()); 
            particles(i).setBotAng(particles(index).getBotAng());
        end
    end
    
    
%% Write code to decide how to move next
 % Move to the furthest wall in steps of distance/path_div
% Localisation occurs between every step

    if motionStep < path_div && motionStep ~= 1 
        turn = 0;
        move = round((scan_max-2*hitRange)/path_div);
    else
        scan_shift = botScan;
        average_scan =  zeros(scans, 1);
        for i = 1 : scans
            average_scan(i) = (scan_shift(end) + scan_shift(1) + scan_shift(2) )/3;
            scan_shift = circshift(scan_shift,-1);
        end
        scan_max_ind_old = scan_max_ind;
        [~, scan_max_ind] = max(average_scan);
        scan_max = botScan(scan_max_ind); 
        if scan_max_ind_old == scan_max_ind 
            [scan_max, scan_max_ind] = max(botScan(botScan~=max(botScan))); % if so choose second largest distance
        end
        turn = angle_interval_bot(scan_max_ind);
        move = round((scan_max-hitRange)/path_div);
        if turn >= pi %choose shortest distance to turn
            turn = turn - 2*pi;
        end
    end
    motionStep = motionStep + 1;

    if min(botScan) < hitRange
        motionStep = 1;
        [~, scan_min_ind] = min(botScan);
        if scan_min_ind < scans/2
            turn = angle_interval_bot(ceil(scan_min_ind+(scans/2)));
        elseif scan_min_ind == scans/2
            turn = pi/2;
        else
            turn = angle_interval_bot(ceil(scan_min_ind-(scans/2)));
        end
        move = hitRange*1.5;
    end

    %botSim.turn(turn); 
    %botSim.move(move);
    movement(turn,move);
    for i =1:num
        particles(i).turn(turn); 
        particles(i).move(move); 
        if particles(i).insideMap == 0 
           particles(i).randomPose(hitRange);
        end
        pose(i,1:2) = particles(i).getBotPos();
        pose(i,3) = mod(particles(i).getBotAng(),2*pi);
    end
    
    %% Write code to check for convergence
    tol = 1;
    unique_clusters = 15;
    [cluster,indC] = uniquetol(pose(:,1:3),tol,'ByRows',true,'OutputAllIndices',true,'DataScale',[1,1,(180/pi)]);
    if size(cluster) <= 1.4*unique_clusters
        [~,weight_max_ind] = max(cellfun('size', indC, 1)); %find the max cell size
        guess(1:3) = cluster(weight_max_ind,:);
        guess(3) = mod(guess(3), 2*pi); 
        converged = 1;
    else
        guess = NaN; %if clusters number not small enough
    end  
         
end

end

