
function [botSim, botEst, particles, weights, lost] = real_localise(botSim, map, target, drawing, debug,left,right,scan)

%Initialize Variables
%botSim = BotSim(map);
tol = 2.5;
unique_clusters = 10;


modifiedMap = map;
mapArea = polyarea(map(:,1),map(:,2)); %Find area of map
num = 550;
wallClearance = 16; % wall clearance of bot
numberScans = 4; % number of ultrasound scans
botSim.setScanConfig(botSim.generateScanConfig(numberScans));
weights = zeros(num,1);
d = zeros(numberScans,1);
particle_data = zeros(num,3);
lost = 0;
move_it = 1;
reverse_flag = 0;
scan_max_ind = NaN;
steps = NaN;
distance_per_length = 7; %distance travelled between localizations
turn_power = 20;


% NXT Motor calibration
full_rot = 820; % turn(780) = 360 degrees
full_dis = 28; % move(28) = 1cm forward

%Noise as StdDev
sensor_noise = 0; %Constant
motion_noise = 0;%0.005; %Proportional
turning_noise = 0;%0.001; %Proportional


angle_interval_bot = (0:(360/numberScans):360-(360/numberScans))...
    .* (pi()/180); %create array of possible ultrasound angles

particles(num,1) = BotSim; %generate some random particles inside the map
for ii = 1:num
    particles(ii) = BotSim(modifiedMap);
    particles(ii).setScanConfig(botSim.generateScanConfig(numberScans));
    particles(ii).randomPose(wallClearance/3);
    particles(ii).setSensorNoise(sensor_noise);
    particles(ii).setTurningNoise(turning_noise);
    particles(ii).setMotionNoise(motion_noise);
end

%% Localisation code
maxNumOfIterations = 10;
n = 0;
converged =0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %particle filter loop

    n = n+1;

    %% Update and Score Particles
    % Perform real scan
    % remove values outside of 0 and 120 (max possible distance in map)
    % remove the value 83 - bizzare reoccurance
    % replace invalid values with average of adjacent values

    [botScan, ~] = ultraScanNew(scan,40,numberScans);

    for ii = 1 : size(botScan)
        if botScan(ii) < 0 || botScan(ii) > 120
            try %for all values except 1st or last
                botScan(ii) = (botScan(ii-1) + botScan(ii+1))/2; %average of adjancent
            catch
                switch ii
                    case 1 %exception for first value
                        botScan(ii) = (botScan(ii+1) + botScan(end))/2;
                    case size(botScan) %exception for last value
                        botScan(ii) = (botScan(1) + botScan(end-1))/2;
                end
            end
        end
    end

    %disp(botScan) %display real scan

    for ii = 1 : num
        [particle_scan, crossing_points] = particles(ii).ultraScan; %particle scan
        %          [SD_scan] = particle_scan_noise(particles(ii),particle_scan,crossing_points,numberScans);
        particle_scan = particle_scan + 1*randn(numberScans,1);
        for jj = 1 : numberScans
            d(jj) = sqrt(sum((particle_scan-botScan).^2)); %euclidean distance
            particle_scan = circshift(particle_scan,-1); %repeat at every orientation
        end
        [min_d, min_d_ind] = min(d); %find minimum euclidean distance (ED) to select correct orientation
        %         if n <
        turn = (min_d_ind-1)*(2*pi()/numberScans); %set particle turning distance
        %         end
        weights(ii) = 1/min_d; %use min ED of selected orientation to obtain weightings
        particles(ii).turn(turn); %Move particles to correct orientation
    end


    weights = weights/sum(weights); %normalize

    %% Resampling - Resampling Wheel

    %Initialize variables
    index = randi([1, num-1]);  %random number for initial starting point on wheel
    beta = 0;
    max_weight = max(weights);
    for ii = 1 : num
        beta = beta + rand(1)*2*max_weight; %Add random amount to beta
        while beta > weights(index) %Resample for selected particles
            beta = beta - weights(index);
            index = rem((index+1),num)+1; %Find remainder of index over number of particles
            weights(ii) = weights(index); %Set weights
            particles(ii).setBotPos(particles(index).getBotPos()); %Set Bot Position
            particles(ii).setBotAng(particles(index).getBotAng());%Set Bot Angle
        end
    end
    hold off
    botSim.drawMap()
    hold on
    for ii =1:num
        particles(ii).drawBot(weights(ii)*1000);
    end


    %% Movement
    % Move to the furthest wall in steps of constant length
    % Localisation occurs between every step
    % distance_per_step - set during intialisation
    %
    if move_it <= steps && move_it ~= 1 %for moving each step
        turn = 0;
        move = distance_per_length;
    else
        scan_shift = botScan;
        %         for ii = 1 : numberScans
        %             average_scan(ii) = (scan_shift(end) + scan_shift(1) + scan_shift(2) )/3; %find average
        %             scan_shift = circshift(scan_shift,-1); %circular shift
        %         end
        average_scan = botScan;
        disp(average_scan);
        scan_max_ind_old = scan_max_ind;
        [~, scan_max_ind] = max(average_scan); %turn by selecting max average
        scan_max = botScan(scan_max_ind); %find distance to travel from real scan
        if scan_max_ind_old == scan_max_ind %check you're not traveling 180 degrees
            [scan_max, scan_max_ind] = max(botScan(botScan~=max(botScan))); % if so choose second largest distance
        end
        turn = angle_interval_bot(scan_max_ind);
        move = distance_per_length;
        steps  = floor((scan_max-wallClearance)/distance_per_length);
        if turn >= pi() %choose shortest distance to turn
            turn = turn - 2*pi();
        end
    end
    move_it = move_it + 1;

    if min(botScan) <= wallClearance && move_it ~= 2 % If within wall clearance begin evasive action
        disp('CLOSE TO WALL!!')
        move_it = 1;
        [~, scan_min_ind] = min(botScan);
        [move, turn] = avasive_action(scan_min_ind,full_dis,full_rot,left,right);
        reverse_flag = 1;
    end

    % NXT send movement
    if round(turn) ~=0 && ~reverse_flag%if statement to remove exception
        if turn >= 0 %Set motor powers for rotation
            left.Power = -turn_power;
            right.Power = turn_power;
        else
            left.Power = turn_power;
            right.Power = -turn_power;
        end
        left.TachoLimit = abs(round(full_rot*(turn/(2*pi()))));
        right.TachoLimit = abs(round(full_rot*(turn/(2*pi()))));
        left.SendToNXT();
        right.SendToNXT();
        left.WaitFor();
        right.WaitFor();
        left.Stop('off');
        right.Stop('off');
    end
    if round(move) ~=0 && ~reverse_flag
        left.Power = 40;
        right.Power = 40;
        left.TachoLimit = abs(round(full_dis*move));
        right.TachoLimit = abs(round(full_dis*move));
        left.SendToNXT();
        right.SendToNXT();
        left.WaitFor();
        right.WaitFor();
        left.Stop('off');
        right.Stop('off');
    end
    reverse_flag = 0;

    [~, SD_move_forward, SD_rotation ] = noise_generator(move,turn,0);
    for ii =1:num
        particles(ii).turn(turn + turn*SD_rotation*randn(1)); %move particles
        particles(ii).move(move + move*SD_move_forward*randn(1)); %Move particles
        if particles(ii).insideMap == 0 % if inside map resample
            %         try
            %             y = randsample(1:num,num,true,weights); % Resample particle
            %         catch
            %             weights(ii) = 0; % If resampling fails set weight of particle to 0
            %         end
            %         particles(ii).setBotPos(particles(y(ii)).getBotPos());
            %         particles(ii).setBotAng(particles(y(ii)).getBotAng());
            particles(ii).randomPose(wallClearance/3)
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
        % %                 botEst(1) = mean(particle_data(cell2mat(iA(weight_max_ind)),1));
        %                 botEst(2) = mean(particle_data(cell2mat(iA(weight_max_ind)),2));
        %                 botEst(3) = mean(particle_data(cell2mat(iA(weight_max_ind)),3));
        botEst(1:3) = C(weight_max_ind,:); %retrieve estimate for bot position
        botEst(3) = mod(botEst(3), 2*pi); %retrieve estimate for bot position
        converged = 1;
    else
        %         botEst = NaN; %if not converged
        size(C,1)
        [~,weight_max_ind] = max(cellfun('size', iA, 1)); %find the max cell size
        %                 botEst(1) = mean(particle_data(cell2mat(iA(weight_max_ind)),1));
        %                 botEst(2) = mean(particle_data(cell2mat(iA(weight_max_ind)),2));
        %                 botEst(3) = mean(particle_data(cell2mat(iA(weight_max_ind)),3));
        botEst(1:3) = C(weight_max_ind,:); %retrieve estimate for bot position
        botEst(3) = mod(botEst(3), 2*pi);
        size(iA,1);
    end



    %% Drawing
    if drawing == 1
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots


        plot(target(1),target(2),'*b', 'MarkerSize', 5);
        plot(C(:,1),C(:,2),'.b','MarkerSize', 10)
        if debug == 1 % Plotting all particles is slow, only plots if debug is on
            for ii =1:num
                particles(ii).drawBot(weights(ii)*1000);
            end
        end

        if isnan(botEst(1))
        else
            if botEst(3) <= pi
                heading = [botEst(1), botEst(2);
                    botEst(1) + 10*cos(botEst(3)), botEst(2)+10*sin(botEst(3))];
            elseif botEst(3) > pi && botEst(3) <= 2*pi
                botEst(3) = botEst(3) - 2*pi;
                heading = [botEst(1), botEst(2);
                    botEst(1) + 10*cos(botEst(3)), botEst(2)+10*sin(botEst(3))];
            end
            if converged
                colour = '.r';
            else
                colour = '.m';
            end
            plot(botEst(1), botEst(2), colour, 'MarkerSize', 20);
            plot(heading(:,1), heading(:,2), 'lineWidth',1.5,'Color','r');
        end
    end
end
if n == maxNumOfIterations
    if size(C) <= 1.5*unique_clusters
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
