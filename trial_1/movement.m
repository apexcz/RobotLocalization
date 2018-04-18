function movement (deg, dis )
dis = dis * 40; % Move distance 
deg = deg * 4; % deg degree

if deg >= pi() %choose shortest distance for rotating
    deg = deg - 2*pi();
end
%% Robot rotating %%
if round(deg) ~=0 
    if deg >= 0 
       l_motor = NXTMotor('B', 'Power', -50, 'TachoLimit', abs(deg/2*pi()));
       r_motor = NXTMotor('C', 'Power', 50, 'TachoLimit', abs(deg/2*pi()));
    else
       l_motor = NXTMotor('B', 'Power', 50, 'TachoLimit', abs(deg/2*pi()));
       r_motor = NXTMotor('C', 'Power', -50, 'TachoLimit', abs(deg/2*pi()));
    end
    l_motor.SendToNXT();
    r_motor.SendToNXT();
    l_motor.WaitFor();
    r_motor.WaitFor();
    l_motor.Stop('off');
    r_motor.Stop('off');
end
%% Robot move %%
if round(move) ~=0 
    if move > 0
       l_motor = NXTMotor('B', 'Power', 50, 'TachoLimit', abs(dis));
       r_motor = NXTMotor('C', 'Power', 50, 'TachoLimit', abs(dis));
    else
       l_motor = NXTMotor('B', 'Power', -50, 'TachoLimit', dis);
       r_motor = NXTMotor('C', 'Power', -50, 'TachoLimit', dis);
    end
    l_motor.SendToNXT();
    r_motor.SendToNXT();
    l_motor.WaitFor();
    r_motor.WaitFor();
    l_motor.Stop('off');
    r_motor.Stop('off');
end
end

