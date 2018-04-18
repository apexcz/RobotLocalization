function [move, turn] = avasive_action(scan_min_ind,full_dis,full_rot,left,right)
switch scan_min_ind
    case 1
        move = -5;
        turn = pi;
        left.Power = -40;
        right.Power = -40;
        left.TachoLimit = abs(round(full_dis*move));
        right.TachoLimit = abs(round(full_dis*move));
        left.SendToNXT();
        right.SendToNXT();
        left.WaitFor();
        right.WaitFor();
        left.Stop('off');
        right.Stop('off');
        left.Power = 20;
        right.Power = -20;
        left.TachoLimit = abs(round(full_rot*(turn/(2*pi()))));
        right.TachoLimit = abs(round(full_rot*(turn/(2*pi()))));
        left.SendToNXT();
        right.SendToNXT();
        left.WaitFor();
        right.WaitFor();
        left.Stop('off');
        right.Stop('off');
    case 2
        turn = -pi/2;
        move = 5;
        left.Power = 20;
        right.Power = -20;
        left.TachoLimit = abs(round(full_rot*(turn/(2*pi()))));
        right.TachoLimit = abs(round(full_rot*(turn/(2*pi()))));
        left.SendToNXT();
        right.SendToNXT();
        left.WaitFor();
        right.WaitFor();
        left.Stop('off');
        right.Stop('off');
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
    case 3
        turn = 0 ;
        move = 5;
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
    case 4
        turn = pi/2;
        move = 5;
        left.Power = -20;
        right.Power = 20;
        left.TachoLimit = abs(round(full_rot*(turn/(2*pi()))));
        right.TachoLimit = abs(round(full_rot*(turn/(2*pi()))));
        left.SendToNXT();
        right.SendToNXT();
        left.WaitFor();
        right.WaitFor();
        left.Stop('off');
        right.Stop('off');
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

