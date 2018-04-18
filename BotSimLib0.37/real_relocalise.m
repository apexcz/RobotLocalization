
function [botSim, botEst, particles, weights] = real_relocalise(botSim, map, target, particles, weights, turn, move, drawing, debug,left,right,scan)



%Initialize Variables
%botSim = BotSim(map);
tol = 15;

mapArea = polyarea(map(:,1),map(:,2)); %Find area of map
num = round((-3e-7*mapArea^2)+(0.02*mapArea)+270);
wallClearance = 16; % wall clearance of bot
numberScans = 8; % number of ultrasound scans

% NXT Motor calibration
full_rot = 780; % turn(780) = 360 degrees
full_dis = 28; % move(28) = 1cm forward


%% Movement
if turn >= pi() %choose shortest distance to turn
    turn = turn - 2*pi();
end
if turn >= 0 %Set motor powers for rotation
    left.Power = -40;
    right.Power = 40;
else
    left.Power = 40;
    right.Power = -40;
end

% NXT send movement
if round(turn) ~=0 %if statement to remove exception
    left.TachoLimit = abs(round(full_rot*(turn/(2*pi()))));
    right.TachoLimit = abs(round(full_rot*(turn/(2*pi()))));
    left.SendToNXT();
    right.SendToNXT();
    left.WaitFor();
    right.WaitFor();
    left.Stop('off');
    right.Stop('off');
end
if round(move) ~=0
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

particle_data = zeros(num, 3);

%% Update and Score Particles
% Perform real scan
% remove values outside of 0 and 120 (max possible distance in map)
% remove the value 83 - bizzare reoccurance
% replace invalid values with average of adjacent values

[~, SD_move_forward, SD_rotation ] = noise_generator(move,turn,0);

for ii =1:num
    particles(ii).turn(turn + SD_rotation*randn(1)); %move particles
    particles(ii).move(move + SD_move_forward*randn(1)); %Move particles
    if particles(ii).insideMap == 0 % if inside map resample
        particles(ii).randomPose(wallClearance);
    end
    particle_data(ii,1:2) = particles(ii).getBotPos();
    particle_data(ii,3) = mod(particles(ii).getBotAng(),2*pi);
end

[botScan, ~] = ultraScanNew(scan,40,numberScans);

[SD_scan_forward, ~,~] = noise_generator(0,0,botScan);

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

disp(botScan) %display real scan

for ii = 1 : num
    particle_scan = particles(ii).ultraScan; %particle scan
    particle_scan = particle_scan + SD_scan_forward(:,1).*randn(length(particle_scan),1);
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



%% Convergence
%By using 'uniquetol' clusters of particles can be found by setting...
%'tol' to an appropriate value to increase required accuracy before...
%convergence.
% 'DataScale' is required to scale the angle data to work with the
% single tolerance level

[C,iA] = uniquetol(particle_data(:,1:3),tol,'ByRows',true,'OutputAllIndices',true,'DataScale',[1,1,(180/pi)]);
[~,weight_max_ind] = max(cellfun('size', iA, 1)); %find the max cell size
botEst(1:3) = C(weight_max_ind,:); %retrieve estimate for bot position
botEst(3) = mod(botEst(3), 2*pi); %retrieve estimate for bot position



%% Drawing
if drawing == 1
    hold off; %the drawMap() function will clear the drawing when hold is off
    botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots


    plot(target(1),target(2),'*b', 'MarkerSize', 5);

    if debug == 1
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
        plot(botEst(1), botEst(2), '.r', 'MarkerSize', 20);
        plot(heading(:,1), heading(:,2), 'lineWidth',1.5,'Color','r');
    end
end
