% Var Init
close all
dydx_store_test = [];
wgs84 = wgs84Ellipsoid;

for i = 1:1:100
    
    [gps_time_diff(i),gps_ind]         = min(abs(GPS_TimeTable.Time(:) - LiDAR_TimeTable.Time(i)));
    
    gps_closest_time(i)     = GPS_TimeTable.Time(gps_ind);
    lidar_time_stamp(i)     = LiDAR_TimeTable.Time(i);
    
    % Set vars
    lat(i)                  = GPS_TimeTable.Data(gps_ind,1);
    lon(i)                  = GPS_TimeTable.Data(gps_ind,2);
    alt                     = GPS_TimeTable.Data(gps_ind,3);
    
    % Setting Local Coords for the first thing in the list        
    if i == 1
        
        lat_start           = double(lat(i));
        lon_start           = double(lon(i));
        alt_start           = double(alt);
        origin              = [lat_start lon_start alt_start];
        
        dx = 0; dy = 0; vect(i) = 0;
        dydx_store_test = [dydx_store_test; dx dy alt vect(i)];
        
        speed(i) = 0;
        
        duration_gps(i) = 0;
        duration_lidar(i) = 0;
        
    else

        [dx, dy, ~] = geodetic2ned(lat(i), lon(i), alt, lat_start, lon_start, alt_start, wgs84);
        
        vect(i)                 = sqrt(dx^2 + dy^2);
        
        d_vect(i)               = vect(i) - vect(i-1);
        
        duration_gps(i)         = gps_closest_time(i) - gps_closest_time(i-1);
        duration_lidar(i)       = lidar_time_stamp(i) - lidar_time_stamp(i-1);
        
        speed(i)                = d_vect(i) / duration_gps(i);
        
    end
    
    dydx_store_test = [dydx_store_test; dx dy alt vect(i)];
    
end

figure
tiledlayout(2,2);

nexttile
plot(gps_time_diff)
xlabel('Point Cloud Number')
ylabel('Time (s)')
title('Time stamp difference')

nexttile
plot(vect)
xlabel('Point Cloud Number')
ylabel('Dist (m)')
title('Distance Traveled')

nexttile
plot(duration_gps(2:end),'r.-')
hold on
plot(duration_lidar(2:end),'b.-')
xlabel('Point Cloud Number')
ylabel('Time Stamp (s)')
legend('GPS','LiDAR')
title('DT Check')
hold off

nexttile
plot(speed(2:end))
xlabel('Point Cloud Number')
ylabel('Speed (m/s)')
title('Speed')


figure
tiledlayout(1,2);

nexttile
geoplot(lat,lon,'.','LineWidth', 3)

nexttile
scatter(dydx_store_test(:,1),dydx_store_test(:,2), '.', 'LineWidth', 3)
axis equal





