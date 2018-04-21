clf;        %clears figures
clc;        %clears console
clear;      %clears workspace

addpath(genpath('\\ads.bris.ac.uk\filestore\MyFiles\StudentPG1\co17397\Downloads\RWTHMindstormsNXTv4.07\RWTHMindstormsNXT'));
% initialise the robot
COM_CloseNXT all;  %prepares workspace
h = COM_OpenNXT(); %prepares workspace,  if this fails, there is an issue with your robot (e.g. connectoin, driver, motorControl22 not running)
COM_SetDefaultNXT(h); %sets default handle

movement(0,20)
%% exit
COM_CloseNXT(h); %clean before program exit 