function [move, turn] = findDirection(botScan)
    move = max(botScan);
    turn = 0;
    step = 2 * pi / length(botScan);
    
    disp(botScan);
        
    for i=1:length(botScan)
        if botScan(i) == move
            break
        end
        turn = turn + step;
    end
    move = min([move*0.8 30]);
end