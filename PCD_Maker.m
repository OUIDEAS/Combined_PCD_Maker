% This program takes the timetables that were made in the
% LiDAR_GPS_IMU_Timetable_Maker function and creates a combined point
% cloud. 


clear all; close all; clc;

% Querey for files
% gps_mat     = uigetfile('*.mat','Grab GPS file');
% imu_mat     = uigetfile('*.mat','Grab IMU file');
% lidar_mat   = uigetfile('*.mat','Grab LiDAR file');
%  

gps_mat = 'Data/GPS_TimeTable.mat';
imu_mat = 'Data/IMU_TimeTable.mat';
lidar_mat = 'Data/LiDAR_TimeTable.mat';
   

% Load Files
disp('Loading files...')
load(gps_mat);
load(imu_mat);
load(lidar_mat);
disp('Loading complete!')

% Querey for export location
% export_dir      = uigetdir( '','Grab PCD Export Directory');


%% Get number of point cloud files
num_pc              = length(LiDAR_TimeTable.Data);

close all; clc;

% VAR INIT SECTION

% PCD_ROT_OFFSET      = rotz(-90);
dxdydz              = [];
dxy_store           = [];
unit_vector_ll      = [0.00001 0 0]; %[x y z]
point_coords        = [];
unit_vector         = [1 0 0];


% Variable for converting GPS coordinates into meters
wgs84               = wgs84Ellipsoid;

% For loop for loading each point cloud, and adjusting using the gps
% and imu data

disp('Processing...')

f = waitbar(0,'1','Name','Doing Da Data');

% loop_array = 1:1:num_pc;
% loop_array = 2:30:num_pc;
loop_array = 2:5:num_pc;

for i = loop_array

%         % Safety, I feel better knowing it's here
%         if i > num_pc
%             disp('wow I''m glad I put this here'); break;
%         end


    %% HANDLING GPS

    [gps_time_diff(i),gps_ind]  = min(abs(GPS_TimeTable.Time(:) - LiDAR_TimeTable.Time(i)));
   
    gps_ind                     = gps_ind + 1;
   
    gps_closest_time(i)         = GPS_TimeTable.Time(gps_ind);
    lidar_time_stamp(i)         = LiDAR_TimeTable.Time(i);
        
    % Set vars
    lat(i)                      = GPS_TimeTable.Data(gps_ind,1);
    lon(i)                      = GPS_TimeTable.Data(gps_ind,2);
    alt(i)                      = GPS_TimeTable.Data(gps_ind,3);
    track(i)                    = GPS_TimeTable.Data(gps_ind,4);
    
   
    % Setting Local Coords for the first thing in the list        
    if i == 2
    
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

        [dx(i), dy(i), dz(i)]       = geodetic2ned(lat(i), lon(i), alt(i), lat_start, lon_start, alt_start, wgs84);
        dx(i) = -dx(i);
        dy(i) = -dy(i);
        dz(i) = dz(i);
        vect(i)                     = sqrt(dx(i)^2 + dy(i)^2 + dz(i)^2);
        
        d_vect(i)                   = vect(i) - vect(i-1);
        
        duration_gps(i)             = gps_closest_time(i) - gps_closest_time(i-1);
        duration_lidar(i)           = lidar_time_stamp(i) - lidar_time_stamp(i-1);
        
        speed(i)                    = d_vect(i) / duration_gps(i);
        
        [lat_post(i), lon_post(i), h_post(i)] = ned2geodetic(dx(i), dy(i), dz(i), lat_start, lon_start, alt_start, wgs84);
        
    end
    
%     dxdydz = [dxdydz; dx(i) dy(i) dz(i)];
    
    % Closest IMU time point
    [imu_time_diff(i),imu_ind]  = min(abs(IMU_TimeTable.Time(:) - LiDAR_TimeTable.Time(i)));
    imu_closest_time(i)         = IMU_TimeTable.Time(imu_ind,:);
    
    % Grabbing the Quaternion
    quat_temp                   = [IMU_TimeTable.Data(imu_ind,:)];

    % W X Y Z
    quat                        = quaternion(quat_temp(1), quat_temp(2), quat_temp(3), quat_temp(4));
    
    rot_mat                     = quat2rotm(quat);
    
    euler                       = quat2eul(quat);
    
    new_point                   = unit_vector * rot_mat;
    
    e_lat(i)                       = lat(i) + new_point(1);
    e_lon(i)                       = lon(i) + new_point(2);
    e_h(i)                         = alt(i) + new_point(3);
    
    point_coords                = [point_coords; lat(i) lon(i) e_lat(i) e_lon(i)];
    
    % Setting offset
    PCD_ROT_OFFSET              =  rotz(90);
    ROTATION_UPDATE = rotx(0) * roty(0) * rotz(track(i) + 90);

    
    %% De-bugging time stamps
%     fprintf("\n LiDAR Time stamp: %s \n GPS Time Stamp: %s \n GPS Time Diff: %s \n IMU Time Stamp: %s \n IMU Time Diff: %s \n", LiDAR_TimeTable.Time(i), gps_closest_time(i), gps_time_diff(i), imu_closest_time(i), imu_time_diff(i))

    %% Adjust point cloud

    % Step 1: Load pc
    % Step 2: Add GPS 
    % Step 3: Use rotatepoint to adjust the point cloud
    
    % Loading point cloud into friendly format
    xyzi                = [double(LiDAR_TimeTable.Data(i).Location) double(LiDAR_TimeTable.Data(i).Intensity)];
    
    % Removing nans and infsspeed
    xyzi                = xyzi( ~any( isnan(xyzi) | isinf(xyzi), 2),:);
    
    % Removing zeros
    % Necessary? Put off for now.........
%     
%     % Swapping the X, Y because of how it's orientated on the van
%     xyzi_temp           = [xyzi(:,2) xyzi(:,1) xyzi(:,3) xyzi(:,4)];
%     xyzi                = xyzi_temp;
    
    % Rotating the point cloud STEP 1: Rotate the cloud 90 degrees
    % counter clockwise to line up with the IMU frame
%     for j = 1:1:length(xyzi(:,1))
% %         xyzi(j,1:3)         = xyzi(j,1:3) * PCD_ROT_OFFSET;
%         
%         xyzi(j,1:3)         = [xyzi(2) xyzi(1) xyzi(3)] * PCD_ROT_OFFSET;
%         
%     end

    
    
    % Rotating the point cloud STEP 2: Use the IMU to adjust the
    % orientation of the point cloud
%     xyzi(:,1:3)          = rotatepoint(quat,[xyzi(:,2) xyzi(:,1) xyzi(:,3)]);
    
%     for k = 1:1:length(xyzi(:,1))
%         xyzi(k,1:3)         = Rot_Mat * xyzi(k,1:3)';
%     end

    xyzi(:,1:3)     = [xyzi(:,1), xyzi(:,2), xyzi(:,3)] * ROTATION_UPDATE * PCD_ROT_OFFSET;
    
    % Adding GPS offset
    xyzi(:,1)            = (xyzi(:,1)) + dx(i);
    xyzi(:,2)            = (xyzi(:,2)) + dy(i);
    xyzi(:,3)            = (xyzi(:,3)) + alt(i);
%     xyzi(:,1:3)     = [xyzi(:,1), xyzi(:,2), xyzi(:,3)] * PCD_ROT_OFFSET;
    
    % Storing dx, dy for debugging
    dxy_store = [dxy_store; dx(i) dy(i) alt(i)];
    
    %% Apphend to overall point cloud

    XYZI_Apphend{i}      = xyzi;

    %% Weight bar

    % You heard me
        waitbar(i/(loop_array(end)),f,sprintf('%1.1f',(i/(loop_array(end))*100)))

    clear quat

end

close(f)

disp('Processing complete, exporting array to point cloud object...')


disp('Point cloud object created. Saving to .pcd...')

%     FileName = string(export_dir) + "/Export_Cloud.pcd";

disp('Point cloud saved to .pcd! Plotting figures...')
%%
color_array = [1 0 0 ; 0 1 0 ; 0 0 1 ; 0 1 1 ; 1 0 1 ; 1 1 0 ; 0 0 0];

j = 1;
% count = 1;
% figure
% for i = loop_array
%     
%     if i == length(color_array(:,1))
%         j = 1;
%     end
% 
%     % Displaying the resulting point cloud
%     
%     color_ind = color_array(j,:);
%     
% %     pcshow(Export_Cloud)  
%     scattet_plt = scatter3(XYZI_Apphend{i}(:,1), XYZI_Apphend{i}(:,2), XYZI_Apphend{i}(:,3));
%     scattet_plt.MarkerFaceColor = color_ind;
%     scattet_plt.MarkerEdgeColor = color_ind;
%     scattet_plt.Marker = '.';
%     
%     hold on
%     origin_plt = scatter3(dx(i), dy(i), alt(i));
% %     origin_plt = scatter3(dxy_store(count,1), dxy_store(count,2), 0);
%     origin_plt.MarkerFaceColor = color_ind;
%     origin_plt.MarkerEdgeColor = color_ind;
%     origin_plt.LineWidth = 10;
%     origin_plt.Marker = '*';
% 
%     
%     view([0 0 90])
%     axis equal
%     
%     j = j + 1;
% %     count = count + 1;
%     
%     hold on
% 
% end

% legend

%%

figure

plot(track)

%% PCD

Export_XYZI = [];

figure

for i = 1:1:length(XYZI_Apphend)
        
    if size(XYZI_Apphend{i}) ~= [0,0]
    
        Export_XYZI = [Export_XYZI; XYZI_Apphend{i}(:,1) XYZI_Apphend{i}(:,2) XYZI_Apphend{i}(:,3) XYZI_Apphend{i}(:,4)];
    
    end
    
end

hold on
Export_Cloud    = pointCloud([Export_XYZI(:,1) Export_XYZI(:,2) Export_XYZI(:,3)], 'Intensity', Export_XYZI(:,4));
plot3(dxy_store(:,1),dxy_store(:,2),dxy_store(:,3),'--','Color','c','LineWidth',6)
scatter3(dxy_store(1,1),dxy_store(1,2),dxy_store(1,3),420,'*','MarkerFaceColor','yellow')
scatter3(dxy_store(end,1),dxy_store(end,2),dxy_store(end,3),420,'*','MarkerFaceColor','magenta')
pcshow(Export_Cloud)
hold off

%% Making the plots prettier
gps_time_diff   = nonzeros(gps_time_diff);
vect            = nonzeros(vect);
duration_gps    = nonzeros(duration_gps);
duration_lidar  = nonzeros(duration_lidar);
speed           = nonzeros(speed);
lat             = nonzeros(lat);
lon             = nonzeros(lon); 

figure
tiledlayout(2,2);

nexttile
plot(loop_array,gps_time_diff,'.-')
xlabel('Point Cloud Number')
ylabel('Time (s)')
title('Time stamp difference')

nexttile
plot(loop_array(2:end),vect,'.-')
xlabel('Point Cloud Number')
ylabel('Dist (m)')
title('Distance Traveled')

nexttile
plot(loop_array(2:end),duration_gps,'ro-')
hold on
plot(loop_array(2:end),duration_lidar,'b.-')
xlabel('Point Cloud Number')
ylabel('Time Stamp (s)')
legend('GPS','LiDAR')
title('DT Check')
hold off

nexttile
plot(loop_array(2:end),speed,'.-')
xlabel('Point Cloud Number')
ylabel('Speed (m/s)')
title('Speed')
%%
figure
tiledlayout(1,2);

nexttile
gp = geoplot(lat,lon);
gp.LineWidth = 5;
gp.LineStyle = '-.';

nexttile
scatter(dy,dx, 'LineWidth', 10)
axis equal

% %%
% 
% figure;
% 
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
scatter(dxdydz(:,1),dxdydz(:,2), '.', 'LineWidth', 3)
axis equal

nexttile
geoplot(lat_post, lon_post, '.', 'LineWidth', 3)


disp('End program.')

