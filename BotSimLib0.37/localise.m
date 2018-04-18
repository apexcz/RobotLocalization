function [ botSim ] = localise1( botSim, map, target )
%	Localistation function for simulated robot. Group MMSW-1.
%
%	===== Inputs =====
%	botSim 	- BotSim class
%	map		- list of vertices of map polygon
%	target 	- [x, y] coordinates of the target

%	===== Outputs ====
%	botSim 	- BotSim class

addpath('C:\Program Files\RWTHMindstormsNXT'); % Add NXT files to path
%NXT Setup
COM_CloseNXT all;
h = COM_OpenNXT();
COM_SetDefaultNXT(h);
left = NXTMotor('A');
right = NXTMotor('B');
scan = NXTMotor('C');
OpenUltrasonic(SENSOR_4);
left.Power = 40;
right.Power = 40;
left.SmoothStart = false;
right.SmoothStart = false;
full_rot = 780;
full_dis = 28;

drawing = true; % Outputs graphical displays for results
debug = true; 	% Outputs graphical displays for debugging

if drawing == true
    botSim.drawMap(); 				% Draw map
    botSim.drawBot(3); 				% Draw initial bot position
    plot(target(1),target(2),'*');	% Draw target
end

%% --- Initial localisation ---
lost = 1;
while lost == 1 % Loops through localisation until the bot has an estimated position
    % First localisation - estimates bots position and heading
    [botSim, botEst, particles, weights, lost] = real_localise(botSim, map, target, drawing, debug,left,right,scan);
end

start = [botEst(1), botEst(2)]; % Start position for route planner
start_angle = botEst(3);		% Start heading

%% --- Route planning ---
resolution = 1; 				% Resolution of grid used for route planning
mink = minkowski(12, map, 0);	% Inflates map to avoid wall collisions

% Generates a discretised map for route planning
[mapGrid, limsMin] = inMap(map, mink, resolution, 0);

% Plans a route from the current bot estimation to the target
[botSim, moves] = routePlanSIM(botSim, mapGrid, limsMin, start, target, resolution, drawing);

if moves == 0
    % rescan
    numberScans = 4;
    angle_interval_bot = (0:(360/numberScans):360-(360/numberScans))...
    .* (pi()/180); 
    [botScan, ~] = ultraScanNew(scan,40,numberScans);
    disp('CLOSE TO WALL!!')
    [~, scan_min_ind] = min(botScan);
    if scan_min_ind < numberScans/2
        turn = angle_interval_bot(ceil(scan_min_ind+(numberScans/2)));
    elseif scan_min_ind == numberScans/2
        turn = pi/2;
    else
        turn = angle_interval_bot(ceil(scan_min_ind-(numberScans/2)));
    end
    move = 16*1.2;
    
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

    % head away from closest
    [botSim, moves] = routePlanSIM(botSim, mapGrid, limsMin, start, target, resolution, drawing);
end
% Converts the route into bot moves
[ botSim ] = move_commandREAL( botSim, moves, start_angle, resolution, left, right);

%% --- Relocalisation ---
%distToTarget = sqrt((botEst(1)-target(1))^2 + (botEst(2)-target(2))^2); % Pythagorean distance to target

% while distToTarget > 2.5 % Loops until the estimate bot is close enough to the target

% 	Scans and generates an updated bot estimate after moving
%     [botSim, botEst, particles, weights] = real_relocalise(botSim, map, target, particles, weights, Turn(1), Forward(1), drawing, debug,left,right,scan);


%
%     start = [botEst(1), botEst(2)]; % Current bot estimate for route planner
%     start_angle = botEst(3);		% Current heading estimate
%
% % 	Plans a new route from the current bot estimation to the target
%     [botSim, moves] = routePlanSIM(botSim, mapGrid, limsMin, start, target, resolution, drawing);
%
%     n = 30;
%     if length(moves) > n 	% Curtails moves if too long to speed up code
%         moves = moves(1:n);
%     end
%
% % 	Converts the route into bot moves
%     [botSim, Turn, Forward] = move_commandSIM( botSim, moves, start_angle, resolution);

% 	Calculates a new distance to the target
%     distToTarget = sqrt((botEst(1)-target(1))^2 + (botEst(2)-target(2))^2);
% end
end
