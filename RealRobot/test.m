addpath(genpath('C:\RWTHMindstormsNXTv4.07'));


COM_CloseNXT all;  %prepares workspace
h = COM_OpenNXT(); %prepares workspace,  if this fails, there is an issue with your robot (e.g. connectoin, driver, motorControl22 not running)
COM_SetDefaultNXT(h); %sets default handle

NXT_PlayTone(440, 500); %(freq,duration)

scanNum = 4;

botReal = BotReal();
% botReal = botReal.setScan(scanNum);
% botReal.ultraScan()

botReal.move(350);
% botReal.ultraScan()
