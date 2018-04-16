% This is for demonstrating the robot's capabilities.
% Before you run this, make sure that 
%- the touch sensor is plugged to Port 1
%- the ultrasound sensor is plugged to Port 4
%- a motor is plugged to Port C
%- the robot is connected via USB and you are running motorControl22 on it
%
% lets play...

%An example of the ultrasound scanning function
clc;        %clears console
clear;      %clears workspace
addpath(genpath('\\ads.bris.ac.uk\filestore\MyFiles\StudentPG1\co17397\Downloads\RWTHMindstormsNXTv4.07\RWTHMindstormsNXT'));
% initialise the robot
COM_CloseNXT all;  %prepares workspace
h = COM_OpenNXT(); %prepares workspace,  if this fails, there is an issue with your robot (e.g. connectoin, driver, motorControl22 not running)
COM_SetDefaultNXT(h); %sets default handle

%%%%%%%%%%%%%%%%%%%%%%%%%%% SENSORS (ports 1-4) %%%%%%%%%%%%%%%%%%%%%%
%% Now we are ready for our first command
NXT_PlayTone(440, 500); %(freq,duration)

% Lets try out the touch sensor
OpenSwitch(SENSOR_1); %open push sensor on port 1 (only needed once for initialisation)
GetSwitch(SENSOR_1) %get reading 1 or 0

% Lets measure distance

mA = NXTMotor('A')  %sensor motor connected to port A
[sensor_scan, ~] = ultraScan(mA,100,4);
disp(sensor_scan)
% You can also read the current motor angle 


%%%%%%%%%%%%%%%%%%%%%%% MOTORS (ports 1-3) %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% lets init a motor object:
mC = NXTMotor('C')  %motor connected to port A

%% lets change motor properties:
mC.Power = 50; %select power output [-100,100]

%% Move motor
mC.SendToNXT(); %move motor

%% Two ways to stop motor:
%mA.Stop('brake') %stops and holds position
mC.Stop('off') %stops and coasts

%% You can also request a specific number of turns:
mC.TachoLimit = 360; %in degrees
mC.SendToNXT(); 

%% Compose a comand:
mC = NXTMotor('C', 'Power', -60, 'TachoLimit', 720); % two turns with speed 60, use +/- values to alter direction
mC.SendToNXT(); 

%% To ensure two successive commands are performed use WaitFor()
mC = NXTMotor('C', 'Power', -50, 'TachoLimit', 360);
mC.SendToNXT();
mC.WaitFor(); % without this the robot will beep in panic!
mC = NXTMotor('C', 'Power', 50, 'TachoLimit', 360);
mC.SendToNXT();
mC.WaitFor(); % without this the motor will barely move!
mC.Stop('off');

mA = NXTMotor('A')  %sensor motor connected to port A
[sensor_scan, ~] = ultraScan(mA,100,4);
disp(sensor_scan)
% You can also read the current motor angle 
data = mC.ReadFromNXT() %reads state of motor 

%[sensor_scan, ~] = ultraScan(mA,90,5);
%disp(sensor_scan)
%% You can also read the current motor angle 
data = mC.ReadFromNXT() %reads state of motor 

%% exit
COM_CloseNXT(h); %clean before program exit 
