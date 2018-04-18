function [SD_scan_straight, SD_move_forward, SD_rotation ] = noise_generator(move,turn,scan)

%%% Scanning noise %%%

% straight on
SD_scan_straight = (-0.002*(scan).^2)+(0.1507*scan)-0.6652;

% 45 degree
SD_scan_45 = (-0.0002*(scan).^3)+(0.0246*(scan).^2)-(1.0875*scan)+17.85;


%%% Movement noise %%%

% forward noise
SD_move_forward = 0.0092*move+0.0624;

% sideways noise
SD_move_side = 1e-5*move^3-0.0004*(move)^2+0.0078*move+0.0285;

%%% Rotation Noise %%%

SD_rotation = deg2rad(0.0105*rad2deg(turn)+0.08423);


% SD_scan_straight = SD_scan_straight/100;
% SD_move_forward = SD_move_forward/100;
%  SD_rotation = SD_rotation/10;

end




