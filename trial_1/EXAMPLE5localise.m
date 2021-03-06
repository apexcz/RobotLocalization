clf;        %clears figures
clc;        %clears console
clear;      %clears workspace

addpath(genpath('\\ads.bris.ac.uk\filestore\MyFiles\StudentPG1\co17397\Downloads\RWTHMindstormsNXTv4.07\RWTHMindstormsNXT'));
% initialise the robot
COM_CloseNXT all;  %prepares workspace
h = COM_OpenNXT(); %prepares workspace,  if this fails, there is an issue with your robot (e.g. connectoin, driver, motorControl22 not running)
COM_SetDefaultNXT(h); %sets default handle

axis equal; %keeps the x and y scale the same
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
map2 = [0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80]; %long map
map3= [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0];

% botSim = BotSim(map,[0.01,0.005,0]);  %sets up a botSim object a map, and debug mode on.
botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.
botSim.drawMap();
drawnow;
botSim.randomPose(10); %puts the robot in a random position at least 10cm away from a wall
%target = botSim.getRndPtInMap(10);  %gets random target.
target = [20,40];
tic %starts timer

%your localisation function is called here.
localise(map,target); %Where the magic happens

resultsTime = toc %stops timer

% fprintf('actual = %.4f , %.4f guessed = %.4f , %.4f \n', actual(:,1),actual(:,2),guessed(:,1),guessed(:,2));
%calculated how far away your robot is from the target.
%resultsDis =  distance(target, returnedBot.getBotPos())

%% exit
COM_CloseNXT(h); %clean before program exit 
