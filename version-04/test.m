addpath(genpath('C:\RWTHMindstormsNXTv4.07'));


COM_CloseNXT all;  %prepares workspace
h = COM_OpenNXT(); %prepares workspace,  if this fails, there is an issue with your robot (e.g. connectoin, driver, motorControl22 not running)
COM_SetDefaultNXT(h); %sets default handle

NXT_PlayTone(440, 500); %(freq,duration)

scanNum = 4;

botReal = BotReal();
botReal = botReal.setScan(scanNum);
botReal.ultraScan();

botReal.move(350);
botReal.ultraScan()

map=[0,0;66,0;66,44;44,44;44,66;110,66;110,110;0,110];  %default map

bot = BotSim(map);  %each particle should use the same map as the botSim object
bot.setBotPos([66 60]);
disp(bot.insideMap());