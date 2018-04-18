function [ botSim ] = movement( botSim, left, right, turn, move )

% NXT Motor calibration
full_rot = 780; % turn(780) = 360 degrees
full_dis = 28; % move(28) = 1cm forward

if turn >= pi() %choose shortest distance to turn
    turn = turn - 2*pi();
end
if turn >= 0 %Set motor powers for rotation
    left.Power = -40;
    right.Power = 40;
else
    left.Power = 40;
    right.Power = -40;
end

% NXT send movement
if round(turn) ~=0 %if statement to remove exception
    left.TachoLimit = abs(round(full_rot*(turn/(2*pi()))));
    right.TachoLimit = abs(round(full_rot*(turn/(2*pi()))));
    left.SendToNXT();
    right.SendToNXT();
    left.WaitFor();
    right.WaitFor();
    left.Stop('off');
    right.Stop('off');
end
if round(move) ~=0
    left.Power = 40;
    right.Power = 40;
    left.TachoLimit = abs(round(full_dis*move));
    right.TachoLimit = abs(round(full_dis*move));
    left.SendToNXT();
    right.SendToNXT();
    left.WaitFor();
    right.WaitFor();
    left.Stop('off');
    right.Stop('off');
end


end

