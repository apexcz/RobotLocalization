function [botSim, botEst, particles, weights, lost] = sim_localiseSIM(botSim, map, target, drawing, debug)
% sim_localise
%
%	===== Inputs =====
%	botSim 	- BotSim class
%	map		- list of vertices of map polygon
%	target 	- [x, y] coordinates of the target
%	drawing - outputs a graphical result
%	debug 	- outputs a detailed graphical output for debugging
%
%	===== Outputs ====
%	botSim 		- BotSim class
%	botEst 		- Estimated bot position and heading
%	particles	- Array of all the particles generated
%	lost		- Flag if bot estimate could not be found

%Initialize Variables
tol = 5;
unique_clusters=30;


modifiedMap = map;
mapArea = polyarea(map(:,1),map(:,2)); %Find area of map
num = round((-3e-7*mapArea^2)+(0.02*mapArea)+270);
wallClearance = 16; % wall clearance of bot
numberScans = 8; % number of ultrasound scans
botSim.setScanConfig(botSim.generateScanConfig(numberScans));
weights = zeros(num,1);
d = zeros(numberScans,1);
particle_data = zeros(num,3);
lost = 0;
move_it = 1;
scan_max_ind = NaN;
steps = NaN;
distance_per_length = 5; %distance travelled between localizations

%Noise as StdDev
sensor_noise = 1; %Constant
motion_noise = 0.01; %Proportional
turning_noise = 0.005; %Proportional

angle_interval_bot = (0:(360/numberScans):360-(360/numberScans))...
    .* (pi()/180); %create array of possible ultrasound angles

particles(num,1) = BotSim; %generate some random particles inside the map
for ii = 1:num
    particles(ii) = BotSim(modifiedMap);
    particles(ii).setScanConfig(botSim.generateScanConfig(numberScans))
    particles(ii).randomPose(wallClearance);
    particles(ii).setSensorNoise(sensor_noise)
    particles(ii).setTurningNoise(turning_noise)
    particles(ii).setMotionNoise(motion_noise)
end

%% Localisation code
maxNumOfIterations = 15;
n = 0;
converged =0;
while(converged == 0 && n < maxNumOfIterations) %particle filter loop
    %% Update and Score Particles
    n = n+1;
    botScan = botSim.ultraScan(); %botSim Scan

    for ii = 1 : num
        particle_scan = particles(ii).ultraScan; %particle scan
        for jj = 1 : numberScans
            d(jj) = sqrt(sum((particle_scan-botScan).^2)); %euclidean distance
            particle_scan = circshift(particle_scan,-1); %repeat at every orientation
        end
        [min_d, min_d_ind] = min(d); %find minimum euclidean distance (ED) to select correct orientation
        turn = (min_d_ind-1)*(2*pi()/numberScans); %set particle turning distance
        weights(ii) = 1/min_d; %use min ED of selected orientation to obtain weightings
        particles(ii).turn(turn) %Move particles to correct orientation
    end
    weights = weights/sum(weights); %normalize

    %% Resampling - Resampling Wheel

    %Initialize variables
    index = randi([1, num-1]);  %random number for initial starting point on wheel
    beta = 0;
    max_weight = max(weights);
    for ii = 1 : num
        beta = beta + rand(1)*2*max_weight; %Add a random number between 0 and max twice the max weight
        while beta > weights(index) %Only resample certain particles
            beta = beta - weights(index); %Calculate the remainder of index/num
            index = rem((index+1),num)+1;
            weights(ii) = weights(index);%Update weights
            particles(ii).setBotPos(particles(index).getBotPos()); %Update particle position
            particles(ii).setBotAng(particles(index).getBotAng()); %Update particle angle
        end
    end

    %% Movement
    % Move to the furthest wall in steps of distance/path_div
    % Localisation occurs between every step

    if move_it < path_div && move_it ~= 1 % If not first iteration or less than path division
        turn = 0; % this is the step phase
        move = round((scan_max-2*wallClearance)/path_div); % move a fraction of the measured distance
    else
        scan_shift = botScan;
        % This loop ensures that a corner is not hit by averaging out groups of scans
        % the average scan then defines the angle in which it turns...
        % ...stops robot from protruding corners
        average_scan =  zeros(numberScans, 1);
        for ii = 1 : numberScans
            average_scan(ii) = (scan_shift(end) + scan_shift(1) + scan_shift(2) )/3; %find average
            scan_shift = circshift(scan_shift,-1); %circular shift instead of complex indexing
        end
        scan_max_ind_old = scan_max_ind;
        [~, scan_max_ind] = max(average_scan); %turn by selecting max average
        scan_max = botScan(scan_max_ind); %find distance to travel from real scan
        if scan_max_ind_old == scan_max_ind %check you're not traveling 180 degrees
            [scan_max, scan_max_ind] = max(botScan(botScan~=max(botScan))); % if so choose second largest distance
        end
        turn = angle_interval_bot(scan_max_ind);
        move = round((scan_max-wallClearance)/path_div);
        if turn >= pi() %choose shortest distance to turn
            turn = turn - 2*pi();
        end
    end
    move_it = move_it + 1;

    if min(botScan) < wallClearance %if within wall clearance begin evasive action
        disp('CLOSE TO WALL!!')
        move_it = 1;
        [~, scan_min_ind] = min(botScan);
        if scan_min_ind < numberScans/2
            turn = angle_interval_bot(ceil(scan_min_ind+(numberScans/2)));
        elseif scan_min_ind == numberScans/2
            turn = pi/2;
        else
            turn = angle_interval_bot(ceil(scan_min_ind-(numberScans/2)));
        end
        move = wallClearance*1.5;
    end

    botSim.turn(turn); %Move Bot
    botSim.move(move); %Move Bot
    for ii =1:num
        particles(ii).turn(turn); %move particles
        particles(ii).move(move); %Move particles
        if particles(ii).insideMap == 0 % if inside map resample
            try
                y = randsample(1:num,num,true,weights); %resample
            catch
                weights(ii) = 0; % if fail set to 0
            end
            particles(ii).setBotPos(particles(y(ii)).getBotPos())
            particles(ii).setBotAng(particles(y(ii)).getBotAng())
        end
        particle_data(ii,1:2) = particles(ii).getBotPos();
        particle_data(ii,3) = mod(particles(ii).getBotAng(),2*pi);
    end


    %% Convergence
    %By using 'uniquetol' clusters of particles can be found by setting...
    %'tol' to an appropriate value to increase required accuracy before...
    %convergence.
    % 'DataScale' is required to scale the angle data to work with the
    % single tolerance level

    [C,iA] = uniquetol(particle_data(:,1:3),tol,'ByRows',true,'OutputAllIndices',true,'DataScale',[1,1,(180/pi)]);

    if size(C) <= unique_clusters %if the number of clusters is below a certain value = converged
        [~,weight_max_ind] = max(cellfun('size', iA, 1)); %find the max cell size
        botEst(1:3) = C(weight_max_ind,:); %retrieve estimate for bot position
        botEst(3) = mod(botEst(3), 2*pi); %retrieve estimate for bot position
        converged = 1;
    else
        botEst = NaN; %if not converged
    end

    %% Drawing
    if drawing == 1
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawScanConfig();
        botSim.drawBot(10,'r');

        plot(target(1),target(2),'*b', 'MarkerSize', 5);

        if debug == 1 % Plotting all particles is slow, only plots if debug is on
            for ii =1:num
                particles(ii).drawBot(weights(ii)*1000);
            end
        end

        if isnan(botEst(1))
        else
            heading = [botEst(1), botEst(2);
                       botEst(1) + 10*sin(botEst(3)), botEst(2)+10*cos(botEst(3))];
            plot(botEst(1), botEst(2), '.r', 'MarkerSize', 20);
            plot(heading(:,1), heading(:,2), 'lineWidth',1.5,'Color','r');
        end

        hold on
        drawnow;
    end
end

if n == maxNumOfIterations
    if size(C) <= 1.4*unique_clusters
        [~,weight_max_ind] = max(cellfun('size', iA, 1)); %find the max cell size
        botEst(1:3) = C(weight_max_ind,:); %retrieve estimate for bot position
        botEst(3) = mod(botEst(3), 2*pi); %retrieve estimate for bot position
        lost = 0;
        disp('Potential Convergence - Returning Bot Position Estimate')
    else
        lost = 1; %if lost return 'lost' flag as true
        disp('HELP IM LOST!')
    end
end
end
