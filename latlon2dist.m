function [dy, dx] = latlon2dist(dlat,dlon,alat)
    
    % Do Something
    rlat = alat * pi/180;
    m = 111132.09 * ones(size(rlat)) - 566.05 * cos(2 * rlat) + 1.2 * cos(4 * rlat);
    dy = dlat .* m ;
    
    rlat = alat * pi/180;
    p = 111415.13 * cos(rlat) - 94.55 * cos(3 * rlat);
    dx = dlon .* p;

end