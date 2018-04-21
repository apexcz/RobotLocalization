clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map

addpath(genpath('C:\RWTHMindstormsNXTv4.07'));

COM_CloseNXT all;  %prepares workspace
h = COM_OpenNXT(); %prepares workspace,  if this fails, there is an issue with your robot (e.g. connectoin, driver, motorControl22 not running)
COM_SetDefaultNXT(h); %sets default handle

NXT_PlayTone(440, 500); %(freq,duration)

target = [80 80];  %gets random target.

tic %starts timer

%your localisation function is called here.
[returnedBot curBot] = localise(map,target); %Where the magic happens

%% Move
modifiedMap = shrinkMap(map, 10); %you need to do this modification yourself
dijk = Dijkstra(modifiedMap, 100, target);
dijk.init();

while true
    position = curBot.getBotPos();
        
    if calWeight(position, target) < 2
        break;
    end
    
    next = dijk.find(position);
    angle = calAngle(position, next);
    move = distance(position, next);
    turn = mod(angle - curBot.getBotAng(), 2*pi);

    returnedBot.turn(turn);
    returnedBot.move(move);
    curBot.turn(turn);
    curBot.move(move);
    
     hold off; %the drawMap() function will clear the drawing when hold is off
     returnedBot.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
     returnedBot.drawBot(10,'g'); %draw robot with line length 30 and green
     curBot.drawBot(10,'b');
     drawnow;
end


resultsTime = toc %stops timer

%calculated how far away your robot is from the target.
resultsDis =  distance(target, returnedBot.getBotPos())