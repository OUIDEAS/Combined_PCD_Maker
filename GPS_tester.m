%% Clear Workspace
close all
clear all
clc

%% Var Init

% Grabbing the files
gps_data        = uigetfile('*.mat','Grab GPS Data');
imu_data        = uigetfile('*.mat','Grab IMU Data');
lidar_data      = uigetfile('*.mat','Grab LIDAR Data');

% Loading the files
load(gps_data);
load(imu_data);
load(lidar_data);

% The first lidar data point is usually trash, trim it:
LiDAR_TimeTable.Time(1) = [];
LiDAR_TimeTable.Data(1) = [];

% Other variables
unit_vector_ll  = [0.00001 0 0];    %[x y z]
unit_vector_m   = [1 0 0];          %[x y z]
point_coords    = [];
dxdydz          = [];
wgs84           = wgs84Ellipsoid;
track_offset    = -90;


%% Loop

for i = 1:1:length(LiDAR_TimeTable.Time)
    
    %% GPS
    
    % Closest gps time stamp
    [gps_time_diff(i),gps_ind]  = min(abs(GPS_TimeTable.Time(:) - LiDAR_TimeTable.Time(i)));
    
    gps_ind = gps_ind + 1;
    
    gps_closest_time(i)         = GPS_TimeTable.Time(gps_ind);
    
    % LiDAR time stamp
    lidar_time_stamp(i)         = LiDAR_TimeTable.Time(i);
        
    % Get vars
    lat(i)                      = GPS_TimeTable.Data(gps_ind,1); % latitude
    lon(i)                      = GPS_TimeTable.Data(gps_ind,2); % longitude
    alt(i)                      = GPS_TimeTable.Data(gps_ind,3); % altitude
    track(i)                    = GPS_TimeTable.Data(gps_ind,4) + track_offset; % heading (deg from north)

    
    % Setting Local Coords for the first thing in the list        
    if i == 1
        
        % Starting point
        lat_start                   = double(lat(i));
        lon_start                   = double(lon(i));
        alt_start                   = double(alt(i));
        
        % Pseudo starting point for the posision check: The first point
        % could probably be eliminated 
        lat_post(1)                 = double(lat(i));
        lon_post(1)                 = double(lon(i));
        alt_post(1)                 = double(alt(i));
                
        dx(i)                       = 0; 
        dy(i)                       = 0;
        dz(i)                       = 0;
        
        vect(i)                     = 0;
        
        dxdydz                      = [dxdydz; dx(i) dy(i) dz(i)];
        
        vect(i)                     = 0;
        speed(i)                    = 0;
        
        duration_gps(i)             = 0;
        duration_lidar(i)           = 0;
        
    else

        [dx(i), dy(i), dz(i)] = geodetic2ned(lat(i), lon(i), alt(i), lat_start, lon_start, alt_start, wgs84);
        
        vect(i)                     = sqrt(dx(i)^2 + dy(i)^2 + dz(i)^2);
        
        d_vect(i)                   = vect(i) - vect(i-1);
        
        duration_gps(i)             = gps_closest_time(i) - gps_closest_time(i-1);
        duration_lidar(i)           = lidar_time_stamp(i) - lidar_time_stamp(i-1);
        
        speed(i)                    = d_vect(i) / duration_gps(i);
        
        [lat_post(i), lon_post(i), h_post(i)] = ned2geodetic(dx(i), dy(i), dz(i), lat_start, lon_start, alt_start, wgs84);
        
    end
    
    dxdydz = [dxdydz; dx(i) dy(i) dz(i)];
    
    %% IMU
    
    % Closest IMU time point
    [imu_time_diff(i),imu_ind]  = min(abs(IMU_TimeTable.Time(:) - LiDAR_TimeTable.Time(i)));
    imu_closest_time(i)         = IMU_TimeTable.Time(imu_ind,:);
    
    % Grabbing the Quaternion
    quat_temp                   = [IMU_TimeTable.Data(imu_ind,:)];

    % W X Y Z Quaternion
    quat                        = quaternion(quat_temp(1), quat_temp(2), quat_temp(3), quat_temp(4));
    
    
    %% Checking Orientation
    
    % Rotational matrix, used for making the unit vectors for verification
    % purposes
    rot_mat_quat                = quat2rotm(quat);
    rot_mat_head                = rotz(track(i));
    
    % Rotating the unit vectors
    new_point_ll                = unit_vector_ll * rot_mat_quat;
    new_point_m_quat            = unit_vector_m * rot_mat_quat;
    new_point_m_track           = unit_vector_m * rot_mat_head;
    
    % Offsetting the points (lat lon alt)
    off_lat(i)                  = lat(i) + new_point_ll(1);
    off_lon(i)                  = lon(i) + new_point_ll(2);
    off_alt(i)                  = alt(i) + new_point_ll(3);
    
    % Offsetting the points (meters x y z)
    off_mx_quat(i)              = dx(i) + new_point_m_quat(1);
    off_my_quat(i)              = dy(i) + new_point_m_quat(2);
    off_mz_quat(i)              = dz(i) + new_point_m_quat(3);
    
    % Ofsetting the points using heading (meters x y z)
    off_mx_track(i)              = dx(i) + new_point_m_track(1);
    off_my_track(i)              = dy(i) + new_point_m_track(2);
    off_mz_track(i)              = dz(i) + new_point_m_track(3);
    
    
end

%% Checking angle between stuff
    
for i = 2:1:(length(LiDAR_TimeTable.Time) - 1)
    
    % Putting vars into nice arrays
    gps_vect                    = [dx(i), dy(i), dz(i)] - [dx(i-1), dy(i-1), dz(i-1)];
    ori_vect                    = [off_mx_quat(i), off_my_quat(i), off_mz_quat(i)] - [dx(i-1), dy(i-1), dz(i-1)];
    track_vect                  = [off_mx_track(i), off_my_track(i), off_mz_track(i)] - [dx(i-1), dy(i-1), dz(i-1)];
    
    % Getting the angle
    theta_gps_ori(i)                    = atan2(norm(cross(gps_vect, ori_vect)), dot(gps_vect, ori_vect));
    theta_gps_norm(i)                   = atan2(norm(cross(unit_vector_m, gps_vect)), dot(unit_vector_m, gps_vect));
        
end

%% Points and vectors showing orientation using lon lat

figure;
for i = 1:1:length(lat)
    
    % Do something    
%     [lat(i) lon(i) e_lat(i) e_lon(i)];
    line_plot_quat = plot([lon(i) off_lon(i)], [lat(i) off_lat(i)]);
    
    line_plot_quat.Color = 'r';
    
    hold on
    
    point_plot = scatter(lon(i), lat(i), 250);
    
    point_plot.Marker = '.';
    point_plot.MarkerFaceColor = 'b';
    point_plot.MarkerEdgeColor = 'b';

    hold on
    
end

hold off
grid on
axis equal
title('lon lat with ori vectors')

xlim_low        = min([lon, off_lon]) - 0.00001;
xlim_high       = max([lon, off_lon]) + 0.00001;

ylim_low        = min([lat, off_lat]) - 0.00001;
ylim_high       = max([lat, off_lat]) + 0.00001;

xlim([xlim_low xlim_high]);
ylim([ylim_low ylim_high]);

xlabel('lat')
ylabel('lon')


%% Points and vectors showing orientation using meters

figure;
for i = 1:1:length(dx)
    
    % Do something    
%     [lat(i) lon(i) e_lat(i) e_lon(i)];
    line_plot_quat = plot([dy(i) off_my_quat(i)], [dx(i) off_mx_quat(i)]);
    
    line_plot_quat.Color = 'r';
    
    hold on
    
    line_plot_track = plot([dy(i) off_my_track(i)], [dx(i) off_mx_track(i)]);
    
    line_plot_track.Color = 'g';
    
    hold on
    
    point_plot = scatter(dy(i), dx(i), 250);
    
    point_plot.Marker = '.';
    point_plot.MarkerFaceColor = 'b';
    point_plot.MarkerEdgeColor = 'b';

    hold on
    
end

hold off
grid on
axis equal
title('x y with ori vectors')

xlim_low        = min([dy off_my_quat]) - 1;
xlim_high       = max([dy off_my_quat]) + 1;

ylim_low        = min([dx off_mx_quat]) - 1;
ylim_high       = max([dx off_mx_quat]) + 1;

xlim([xlim_low xlim_high]);
ylim([ylim_low ylim_high]);

xlabel('x')
ylabel('y')

%% Plotting theta, the angle between orientation vector and closest distance between gps points

figure;
plot((180/pi).*theta_gps_ori)
grid on
xlabel('Instance')
ylabel('theta (rad)')
ylim([min((180/pi).*theta_gps_ori) max((180/pi).*theta_gps_ori)])

%% Plotting theta, the angle between closest distance between gps points and unit vector (1 unit in x direction)

figure;
plot((180/pi).*theta_gps_norm)
grid on
xlabel('Instance')
ylabel('theta (rad)')
ylim([min((180/pi).*theta_gps_norm) max((180/pi).*theta_gps_norm)])

%% Other Plots

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
scatter(dxdydz(:,1),dxdydz(:,2), '.', 'LineWidth', 3)
axis equal

nexttile
geoplot(lat_post, lon_post, '.', 'LineWidth', 3)





