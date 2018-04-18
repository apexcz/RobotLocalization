function [radii, angles] = ultraScan(scanSpeed,samples)
%motor.stop('Brake'); %cancels any previous movement that may be happening
%motor.ActionAtTachoLimit = 'Brake'; % Want precise movment

motor = NXTMotor('A')  %sensor motor connected to port A;
motor.Power = -scanSpeed; % so it turns counterclockwise
motor.SmoothStart = false; %we want the scan to be as linear as possible
disp('scanning...');
angleIt = 360/samples;
angles = zeros(samples,1); %preallocate
radii = zeros(samples,1); %preallocate
NXT_ResetMotorPosition(2,true)
OpenUltrasonic(SENSOR_4)
pause(1)
for i = 1:samples
    radii(i) = GetUltrasonic(SENSOR_4);
    angles(i) = i*angleIt;
    motor.TachoLimit =angleIt;
    motor.SendToNXT(); %move motor
    motor.WaitFor();    
end

%move the motor back to it's original position
%you could probably get a second scan at this point if you wanted and merge
%the data. Or just stay at this position until you want to scan again and
%turn the other way to stop the cable getting tangled.
disp('returning to original position');
motor.Power = scanSpeed;
motor.TachoLimit = 360;
motor.SendToNXT(); %move motor
motor.WaitFor();
CloseSensor(SENSOR_4);
