%% Initial the env.
clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same

%% Initial the Robotic dervier
addpath(genpath('C:\RWTHMindstormsNXTv4.07'));

COM_CloseNXT all;  %prepares workspace
h = COM_OpenNXT(); %prepares workspace,  if this fails, there is an issue with your robot (e.g. connectoin, driver, motorControl22 not running)
COM_SetDefaultNXT(h); %sets default handle

NXT_PlayTone(440, 500); %(freq,duration)

%% Global Variables
target = [80, 80];
map=[0,0;66,0;66,44;44,44;44,66;110,66;110,110;0,110];  %default map
%map=[0,0;112,0;112,112;44,112;44,66;66,66;66,44;0,44];

botReal = BotReal();

% Sim Bobotic
% botReal = BotSim(map,[0,0,0]);
% botReal.setBotPos([22 22]);
% botReal.setBotAng(0);
% botReal.setScanConfig(botReal.generateScanConfig(8));

%% Loclisation

tic %starts timer

% your localisation function is called here.
returnedBot = localise(botReal, map, target); %Where the magic happens

resultsTime = toc %stops timer


NXT_PlayTone(440, 500);

%% Finish
% calculated how far away your robot is from the target.
% resultsDis =  distance(target, returnedBot.getBotPos())