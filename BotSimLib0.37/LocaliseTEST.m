clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
map = [0,0;66,0;66,44;44,44;44,66;110,66;110,110;0,110];  %default map
%map = [0,0;50,0;50,50;0,50];
% map = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0]; %repeated features
%map = [0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80]; %long map

botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.  
%botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.
botSim.drawMap();
drawnow;

% botSim.randomPose(10);
% botSim.setBotPos([225,55])%puts the robot in a random position at least 10cm away from a wall
target = [88,88] %botSim.getRndPtInMap(10);  %gets random target.

tic %starts timer

%your localisation function is called here.
returnedBot = localise1(botSim,map,target); %Where the magic happens

resultsTime = toc %stops timer

%calculated how far away your robot is from the target.
resultsDis =  distance(target, returnedBot.getBotPos())