function [ botSim ] = move_commandREAL( botSim, moves, start_angle, resolution, left, right)
power_rot = 20;
power_move = 40;
tacho_rot_full = 820;
if start_angle < 0
    start_angle = mod(abs(start_angle - 2*pi),2*pi);
end
    
% rotate to direction 1
if start_angle ~= pi/2;
    if start_angle > pi/2 && start_angle < (3*pi)/2
        left.Power = power_rot;
        right.Power = -power_rot;
        left.SmoothStart = false;
        right.SmoothStart = false;
        left.TachoLimit = round(tacho_rot_full*((start_angle-(pi/2))/(2*pi)));
        right.TachoLimit = round(tacho_rot_full*((start_angle-(pi/2))/(2*pi)));
    elseif start_angle >= 3*pi/2 && start_angle < 2*pi;
        left.Power = -power_rot;
        right.Power = power_rot;
        left.SmoothStart = false;
        right.SmoothStart = false;
        left.TachoLimit = round(tacho_rot_full*((((2*pi)-start_angle)+(pi/2))/(2*pi)));
        right.TachoLimit = round(tacho_rot_full*((((2*pi)-start_angle)+(pi/2))/(2*pi)));
    else
        left.Power = -power_rot;
        right.Power = power_rot;
        left.SmoothStart = false;
        right.SmoothStart = false;
        left.TachoLimit = round(tacho_rot_full*(((pi/2)-start_angle)/(2*pi)));
        right.TachoLimit = round(tacho_rot_full*(((pi/2)-start_angle)/(2*pi)));
    end
    left.SendToNXT();
    right.SendToNXT();
    
    left.WaitFor();
    right.WaitFor();
    
    left.Stop('off');
    right.Stop('off');
end

% set initial values
sum_moves= 1;
ii = 0;
for i = 1:length(moves);
    
    % break on last iteration
    if (length(moves))== ii + sum_moves - 1;
        break
    end
    
    %%% Rotation for first iteration %%%
    if i == 1;
        pause(1)
        if moves(i) < 6 && moves(i) >0
            left.Power = -power_rot;
            right.Power = power_rot;
            left.SmoothStart = false;
            right.SmoothStart = false;
            left.TachoLimit = round((tacho_rot_full/8)*((moves(i))-1));
            right.TachoLimit = round((tacho_rot_full/8)*((moves(i))-1));
        elseif moves >= 6
            left.Power = power_rot;
            right.Power = -power_rot;
            left.SmoothStart = false;
            right.SmoothStart = false;
            left.TachoLimit = round((tacho_rot_full/8)*(9-(moves(i))));
            right.TachoLimit = round((tacho_rot_full/8)*(9-(moves(i))));
        end
        
        left.SendToNXT();
        right.SendToNXT();
        
        left.WaitFor();
        right.WaitFor();
        
        left.Stop('off');
        right.Stop('off');
        
        
        %%% Rotation after first iteration %%%
    else
        
        % adjust iterations
        ii = ii + sum_moves;
        i = ii;
        
        % rotate
        if moves(i-1) < 5;
            if moves(i) > moves(i-1) && moves(i) < (moves(i-1)+5);
                % Turn Left
                left.Power = -power_rot;
                right.Power = power_rot;
                left.SmoothStart = false;
                right.SmoothStart = false;
                diff = abs(moves(i)-moves(i-1));
                if diff > 4;
                    diff = 8 - diff;
                    left.TachoLimit = (tacho_rot_full/8)*diff;
                    right.TachoLimit = (tacho_rot_full/8)*diff;
                else
                    left.TachoLimit = round((tacho_rot_full/8)*diff);
                    right.TachoLimit = round((tacho_rot_full/8)*diff);
                end
            else
                % turn right
                left.Power = power_rot;
                right.Power = -power_rot;
                left.SmoothStart = false;
                right.SmoothStart = false;
                diff = abs(moves(i)-moves(i-1));
                if diff > 4;
                    diff = 8 - diff;
                    left.TachoLimit = round((tacho_rot_full/8)*diff);
                    right.TachoLimit = round((tacho_rot_full/8)*diff);
                else
                    left.TachoLimit = round((tacho_rot_full/8)*diff);
                    right.TachoLimit = round((tacho_rot_full/8)*diff);
                end
            end
        else
            if moves(i) > (moves(i-1)-4) && moves(i) < moves(i-1);
                % turn right
                left.Power = power_rot;
                right.Power = -power_rot;
                left.SmoothStart = false;
                right.SmoothStart = false;
                diff = abs(moves(i)-moves(i-1));
                if diff > 4;
                    diff = 8 - diff;
                    left.TachoLimit = round((tacho_rot_full/8)*diff);
                    right.TachoLimit = round((tacho_rot_full/8)*diff);
                else
                    left.TachoLimit = round((tacho_rot_full/8)*diff);
                    right.TachoLimit = round((tacho_rot_full/8)*diff);
                end
            else
                % turn left
                left.Power = -power_rot;
                right.Power = power_rot;
                left.SmoothStart = false;
                right.SmoothStart = false;
                diff = abs(moves(i)-moves(i-1));
                if diff > 4;
                    diff = 8 - diff;
                    left.TachoLimit = round((tacho_rot_full/8)*diff);
                    right.TachoLimit = round((tacho_rot_full/8)*diff);
                else
                    left.TachoLimit = round((tacho_rot_full/8)*diff);
                    right.TachoLimit = round((tacho_rot_full/8)*diff);
                end
            end
        end
        left.SendToNXT();
        right.SendToNXT();
        
        left.WaitFor();
        right.WaitFor();
        
        left.Stop('off');
        right.Stop('off');
        
    end
    
    %%% movement %%%
    
    % sum moves
    ii = i;
    sum_moves = 0;
    n = moves(i);
    while moves(i)== n;
        sum_moves = sum_moves+1;
        if i == length(moves)
            break
        end
        
        i=i+1;
    end
    i = ii;
    
    left.Power = power_move;
    right.Power = power_move;
    left.SmoothStart = false;
    right.SmoothStart = false;
    
    %diagonal add distance
    if n == 2 || n == 4 || n == 6 || n == 8;
        left.TachoLimit = 40*resolution*sum_moves;
        right.TachoLimit = 40*resolution*sum_moves;
    else
        left.TachoLimit = 28*resolution*sum_moves;
        right.TachoLimit = 28*resolution*sum_moves;
    end
    
    left.SendToNXT();
    right.SendToNXT();
    
    left.WaitFor();
    right.WaitFor();
    
    left.Stop('off');
    right.Stop('off');
    
end
end




