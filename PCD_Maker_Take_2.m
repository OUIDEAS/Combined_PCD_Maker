% This program takes the timetables that were made in the
% LiDAR_GPS_IMU_Timetable_Maker function and creates a combined point
% cloud. 


clear all; close all; clc;

% Querey for files
gps_mat             = uigetfile('*.mat','Grab GPS file');
imu_mat             = uigetfile('*.mat','Grab IMU file');
lidar_mat           = uigetfile('*.mat','Grab LiDAR file');
   
% Load Files
disp('Loading files...')
load(gps_mat);
load(imu_mat);
load(lidar_mat);
disp('Loading complete!')

% Querey for export location
export_dir          = uigetdir( '','Grab PCD Export Directory');

%% Get number of point cloud files
num_pc              = length(LiDAR_TimeTable.Data);

close all; clc;

% VAR INIT SECTION

PCD_ROT_OFFSET      = rotz(-90);
dxdyda              = [];
dxy_store           = [];
unit_vector         = [0.00001 0 0]; %[x y z]
point_coords        = [];

% Variable for converting GPS coordinates into meters
wgs84               = wgs84Ellipsoid;

% For loop for loading each point cloud, and adjusting using the gps
% and imu data

disp('Processing...')

f = waitbar(0,'1','Name','Doing Da Data');

for i = 1:1:num_pc
    
    %% Load GPS
    
    [gps_time_diff(i),gps_ind]  = min(abs(GPS_TimeTable.Time(:) - LiDAR_TimeTable.Time(i)));
    
    gps_closest_time(i)         = GPS_TimeTable.Time(gps_ind);
    
    % Set vars
    lat(i)                      = GPS_TimeTable.Data(gps_ind,1);
    lon(i)                      = GPS_TimeTable.Data(gps_ind,2);
    alt(i)                      = GPS_TimeTable.Data(gps_ind,3);
    
    if i == 1 % Setting Local Coords for the first thing in the list
        
        lat_start                   = double(lat(i));
        lon_start                   = double(lon(i));
        alt_start                   = double(alt(i));
        
        lat_post(i)                 = double(lat(i));
        lon_post(i)                 = double(lon(i));
        alt_post(i)                 = double(alt(i));
                
        dx(i)                       = 0; 
        dy(i)                       = 0;
        da(i)                       = 0;
        
        dxdyda                      = [dxdyda; dx(i) dy(i) da(i)];
        
        vect(i)                     = 0;
        speed(i)                    = 0;
        
        duration_gps(i)             = 0;
        
    else

        [dx(i), dy(i), da(i)]       = geodetic2ned(lat(i), lon(i), alt(i), lat_start, lon_start, alt_start, wgs84);
        
        dxdyda                      = [dxdyda; dx(i) dy(i) da(i)];
        
        vect(i)                     = sqrt(dx(i)^2 + dy(i)^2);
        
        d_vect(i)                   = vect(i) - vect(i-1);
        
        duration_gps(i)             = gps_closest_time(i) - gps_closest_time(i-1);
%         duration_lidar(i)           = lidar_time_stamp(i) - lidar_time_stamp(i-1);
        
        speed(i)                    = d_vect(i) / duration_gps(i);
        
        [lat_post(i), lon_post(i), h_post(i)] = ned2geodetic(dx(i), dy(i), da(i), lat_start, lon_start, alt_start, wgs84);
        
    end
    
    
    
    %% Load IMU
    
    %% Load LiDAR
    
    %% Orientation Check
    
%     rot_mat                     = quat2rotm(quat);
%     
%     new_point                       = unit_vector * rot_mat;
%     
%     e_lat(i)                        = lat(i) + new_point(1);
%     e_lon(i)                        = lon(i) + new_point(2);
%     e_h(i)                          = alt(i) + new_point(3);
    
    %% Weight bar

    % You heard me
    waitbar(i/(loop_array(end)),f,sprintf('%1.1f',(i/(loop_array(end))*100)))
    
end

close(f)

%% GPS Check
figure
tiledlayout(1,3);

nexttile
geoplot(lat,lon,'.','LineWidth', 3)

nexttile
scatter(dxdyda(:,1),dxdyda(:,2), '.', 'LineWidth', 3)
axis equal

nexttile
geoplot(lat_post, lon_post, '.', 'LineWidth', 3)

%% Orientation Check
% 
% figure;
% for i = 1:1:length(lat)
%     
%     % Do something    
% %     [lat(i) lon(i) e_lat(i) e_lon(i)];
%     line_plot = plot([lon(i) e_lon(i)], [lat(i) e_lat(i)]);
%     
%     line_plot.Color = 'r';
%     
%     hold on
%     
%     point_plot = scatter(lon(i), lat(i), 250);
%     
%     point_plot.Marker = '.';
%     point_plot.MarkerFaceColor = 'b';
%     point_plot.MarkerEdgeColor = 'b';
% 
%     hold on
%     
% end
% 
% hold off
% grid on
% axis equal
% 
% xlim_low        = min([lon, e_lon]) - 0.00001;
% xlim_high       = max([lon, e_lon]) + 0.00001;
% 
% ylim_low        = min([lat, e_lat]) - 0.00001;
% ylim_high       = max([lat, e_lat]) + 0.00001;
% 
% xlim([xlim_low xlim_high]);
% ylim([ylim_low ylim_high]);






