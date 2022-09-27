% Var Init
close all
clear all
clc
dydx_store_test = [];
wgs84 = wgs84Ellipsoid;

gps_data = uigetfile('*.mat','Grab GPS Data');
imu_data = uigetfile('*.mat','Grab IMU Data');
lidar_data = uigetfile('*.mat','Grab LIDAR Data');

load(gps_data);
load(imu_data);
load(lidar_data);

unit_vector = [0.00001 0 0]; %[x y z]
point_coords = [];

for i = 1:1:length(LiDAR_TimeTable.Time)
    
    [gps_time_diff(i),gps_ind]  = min(abs(GPS_TimeTable.Time(:) - LiDAR_TimeTable.Time(i)));
    
    gps_closest_time(i)         = GPS_TimeTable.Time(gps_ind);
    lidar_time_stamp(i)         = LiDAR_TimeTable.Time(i);
    
    gps_ind = i;
    
    % Set vars
    lat(i)                      = GPS_TimeTable.Data(gps_ind,1);
    lon(i)                      = GPS_TimeTable.Data(gps_ind,2);
    alt(i)                      = GPS_TimeTable.Data(gps_ind,3);
    
    % Setting Local Coords for the first thing in the list        
    if i == 1
        
        lat_start                   = double(lat(i));
        lon_start                   = double(lon(i));
        
        lat_post(1)                 = double(lat(i));
        lon_post(1)                 = double(lon(i));
        
        alt_start                   = double(alt(i));
        alt_post(1)                 = double(alt(i));
        
        origin                      = [lat_start lon_start alt_start];
        
        dx                          = 0; 
        dy                          = 0; 
        vect(i)                     = 0;
        
        dydx_store_test             = [dydx_store_test; dx dy alt(i) vect(i)];
        
        speed(i)                    = 0;
        
        duration_gps(i)             = 0;
        duration_lidar(i)           = 0;
        
    else

        [dy, dx, d_alt] = geodetic2ned(lat(i), lon(i), alt(i), lat_start, lon_start, alt_start, wgs84);
        
        vect(i)                     = sqrt(dx^2 + dy^2);
        
        d_vect(i)                   = vect(i) - vect(i-1);
        
        duration_gps(i)             = gps_closest_time(i) - gps_closest_time(i-1);
        duration_lidar(i)           = lidar_time_stamp(i) - lidar_time_stamp(i-1);
        
        speed(i)                    = d_vect(i) / duration_gps(i);
        
        [lat_post(i), lon_post(i), h_post(i)] = ned2geodetic(dy, dx, d_alt, lat_start, lon_start, alt_start, wgs84);
        
    end
    
    dydx_store_test = [dydx_store_test; dx dy alt(i) vect(i)];
    
    % Closest IMU time point
    [imu_time_diff(i),imu_ind]  = min(abs(IMU_TimeTable.Time(:) - LiDAR_TimeTable.Time(i)));
    imu_closest_time(i)         = IMU_TimeTable.Time(imu_ind,:);
    
    % Grabbing the Quaternion
    quat_temp                   = [IMU_TimeTable.Data(imu_ind,:)];

    % W X Y Z
    quat                        = quaternion(quat_temp(1), quat_temp(2), quat_temp(3), quat_temp(4));
    
    rot_mat                     = quat2rotm(quat);
    
%     euler                       = quat2eul(quat);
    
    new_point                   = unit_vector * rot_mat;
    
    e_lat(i)                       = lat(i) + new_point(1);
    e_lon(i)                       = lon(i) + new_point(2);
    e_h(i)                         = alt(i) + new_point(3);
    
    point_coords                = [point_coords; lat(i) lon(i) e_lat(i) e_lon(i)];
    
end

%%
figure

for i = 1:1:length(lat)
    
    % Do something    
    point_plot.Marker = '.';
    point_plot.LineWidth = 100;

    hold on
    vectarrow([lat(i) lon(i)], [e_lat(i), e_lon(i)])
    hold on
    point_plot = plot(lat(i), lon(i));

    hold on
    
    
end

%%

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
tiledlayout(1,3);

nexttile
geoplot(lat,lon,'.','LineWidth', 3)

nexttile
scatter(dydx_store_test(:,1),dydx_store_test(:,2), '.', 'LineWidth', 3)
axis equal

nexttile
geoplot(lat_post, lon_post, '.', 'LineWidth', 3)





