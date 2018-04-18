function[SD_scan] = particle_scan_noise(particle_no,scan,crossing_points,numberScans)

for ii = 1 : numberScans
    botPos = particle_no.getBotPos();
    x_diff(ii) = abs(botPos(1)-crossing_points(ii,1));
    y_diff(ii) = abs(botPos(2)-crossing_points(ii,2));
    angle(ii) = (atand(y_diff(ii)/x_diff(ii)));
    
    if angle(ii) < 22.5 % straight on
        SD_scan(ii) = -0.002*scan(ii)^2 + 0.1507*scan(ii) - 0.6652;
    else  % 45 degree
        SD_scan(ii) = -0.0002*scan(ii)^3 + 0.0246*scan(ii)^2 - 1.0875*scan(ii) + 17.85;
    end
    if SD_scan(ii) < 0.5
        SD_scan(ii) = 0.5;
    end
    if SD_scan(ii) > 6
        SD_scan(ii) = 6;
    end
end
end
