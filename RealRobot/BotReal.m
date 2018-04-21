classdef BotReal
    %BOTREAL Summary of this class goes here
    %   Detailed  explanation goes here
    
    properties
        scanNumber;
        scanPower;          %power of motor
        movePower;
        moveStep;       %the length of 1cm
        turnStep;
    end
    
    methods
        function bot = BotReal()
            bot.scanNumber = 6;
            bot.scanPower = 20;
            bot.movePower = 50;
            bot.moveStep = 35;
            bot.turnStep = 318.30;
        end
        
        function bot = setScan(bot, scanNum)
            bot.scanNumber = scanNum;
        end
        
        function distances = ultraScan(bot)
            distances = zeros(bot.scanNumber,1);
            step = round(360.0 / bot.scanNumber);
            motor = NXTMotor('A');
            motor.Power = -bot.scanPower;
            
            start = motor.ReadFromNXT();
            
            OpenUltrasonic(SENSOR_4);
            pause(1);
            
            for i=1:bot.scanNumber
                distances(i) = GetUltrasonic(SENSOR_4);
                motor.TachoLimit = step;
                motor.SendToNXT();
                motor.WaitFor();
            end
            
            stop = motor.ReadFromNXT();
            
            motor.Power = bot.scanPower;
            motor.TachoLimit = start.Position - stop.Position;
            motor.SendToNXT();
            motor.WaitFor();
        end
        
        function move(bot, distance)
            if distance == 0
                return
            end
            step = round(abs(distance) * bot.moveStep);
            if distance > 0
                l_motor = NXTMotor('B', 'Power', bot.movePower, 'TachoLimit', step);
                r_motor = NXTMotor('C', 'Power', bot.movePower, 'TachoLimit', step);
            else
                l_motor = NXTMotor('B', 'Power', -bot.movePower, 'TachoLimit', step);
                r_motor = NXTMotor('C', 'Power', -bot.movePower, 'TachoLimit', step);
            end
            
            l_motor.SendToNXT();
            r_motor.SendToNXT();
            l_motor.WaitFor();
            r_motor.WaitFor();
            l_motor.Stop('off');
            r_motor.Stop('off');
        end
        
        function turn(bot, deltaAngle)
            if deltaAngle == 0
                return
            end
            if deltaAngle > pi
                deltaAngle = deltaAngle - 2 * pi;
            end
            
            step = round(abs(deltaAngle) * bot.turnStep);
            if deltaAngle >= 0
                l_motor = NXTMotor('B', 'Power', -bot.movePower, 'TachoLimit', step);
                r_motor = NXTMotor('C', 'Power', bot.movePower, 'TachoLimit', step);
            else
                l_motor = NXTMotor('B', 'Power', bot.movePower, 'TachoLimit', step);
                r_motor = NXTMotor('C', 'Power', -bot.movePower, 'TachoLimit', step);
            end
            
            l_motor.SendToNXT();
            r_motor.SendToNXT();
            l_motor.WaitFor();
            r_motor.WaitFor();
            l_motor.Stop('off');
            r_motor.Stop('off');
        end
    end
    
end

