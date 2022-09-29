% This program takes the timetables that were made in the
% LiDAR_GPS_IMU_Timetable_Maker function and creates a combined point
% cloud. 

%% Clearing Workspace

% If the progress bar is open it will close it before clearing workspace
if exist('f','class')
    close(f)
end

clear all; close all; clc;

%% Opening Data Files

% Querey for files
% gps_mat     = uigetfile('*.mat','Grab GPS file');
% imu_mat     = uigetfile('*.mat','Grab IMU file');
% lidar_mat   = uigetfile('*.mat','Grab LiDAR file');

% Hard code
gps_mat = '/media/autobuntu/chonk/chonk/git_repos/PCD_Map_Maker_2/Data/Plians_1/GPS_TimeTable.mat';
imu_mat = '/media/autobuntu/chonk/chonk/git_repos/PCD_Map_Maker_2/Data/Plians_1/IMU_TimeTable.mat';
lidar_mat = '/media/autobuntu/chonk/chonk/git_repos/PCD_Map_Maker_2/Data/Plians_1/LiDAR_TimeTable.mat';

% Load Files
disp('Loading files...')
load(gps_mat);
load(imu_mat);
load(lidar_mat);
disp('Loading complete!')

%% Obtaining export location

% Querey for export location
% export_dir      = uigetdir( '','Grab PCD Export Directory');

%% Variable Initiation

% Storage index - used for storing all variables. Seperate from the loop
% counter, which looks at the length of the index of the point clouds
store_ind                   = 1;

% Plotting stuff **NO LONGER NEEDED PLOTTING UPDATE REQUIRED PRIOR TO REMOVAL**
dxy_store                   = [];

% bool to determine if we loop through each generated point cloud
loop_bool                   = 0;

% Unit vectors for plotting
unit_vector_m               = [1 0 0];          %[x y z]
unit_vector_ll              = [0.00001 0 0];    %[x y z]

% Variable for converting GPS coordinates into meters
wgs84                       = wgs84Ellipsoid;

% Initilizing variables outside of the main loop for cleanliness

% Distance from the origin in meters
dx_from_origin(store_ind)   = 0; 
dy_from_origin(store_ind)   = 0;
dz_from_origin(store_ind)   = 0;
dxdydz                      = [dx_from_origin(store_ind) dy_from_origin(store_ind) dz_from_origin(store_ind)];

% Distance between the current and previous dx and dy, used to calculate
% speed along with the time stamps from the gps topic
dist(store_ind)             = 0;

% Time between the current and previous dx and dy timestamps, used to
% calculate speed along with the distance
duration_gps(store_ind)     = 0;

% Speed calculated from the distance 
speed_gps(store_ind)        = 0;

% Time difference between the current and previous lidar time stamps
duration_lidar(store_ind)   = 0;

% Frame reference resolution between the LiDAR and the gps/imu

% LiDAR reference frame:    1.584; 0; 1.444 [X Y Z]
LiDAR_Ref_Frame             = [ 1.584; 0; 1.444];

% IMU reference frame:      0.336; 0; -0.046 [X Y Z]
IMU_Ref_Frame               = [0.336; 0; -0.046];

% Correction frame:         LiDAR_Ref_Frame - IMU_Ref_Frame [X Y Z]
gps_lidar_diff              = [(LiDAR_Ref_Frame(1) - IMU_Ref_Frame(1)), (LiDAR_Ref_Frame(2) - IMU_Ref_Frame(2)), (LiDAR_Ref_Frame(3) - IMU_Ref_Frame(3))]; 

% Correction rotation: IMU is rotated -90* relative to the LiDAR
PCD_ROT_OFFSET              = 90;

%% Processing loaded data

disp('Processing...')

% Progress bar
f = waitbar(0,'1','Name','Doing Da Data');

% Caluclating number of point clouds in the LiDAR data mat file
num_pc              = length(LiDAR_TimeTable.Data);

% Creating an array to determine which point clouds are used
% loop_array = 1:1:num_pc;
% loop_array = 2:30:num_pc;
loop_array          = 2:2:16;

% For loop that loads each point cloud, and adjusting using the gps
% and imu data. "i" is the selected point cloud as per the loop_array 
% variable, and store_ind is the indexing variable used to store all other 
% variables

for i = loop_array

    %% Loop Safety
    
    if i > num_pc
        disp('wow I''m glad I put this here'); break;
    end

    %% HANDLING GPS
    
    % Finding the smallest time difference between the current lidar time
    % stamp and the any gps time stamp. The index with the smallest time is
    % selected
    [gps_time_diff(store_ind), gps_ind]  = min(abs(GPS_TimeTable.Time(:) - LiDAR_TimeTable.Time(i)));
    
    % The gps index is increased by one to increase odds of getting a good
    % gps selection
    gps_ind                             = gps_ind + 1;
    
    % The gps and lidar time stamps are saved
    gps_closest_time(store_ind)         = GPS_TimeTable.Time(gps_ind);
    lidar_time_stamp(store_ind)         = LiDAR_TimeTable.Time(i);
        
    % Set vars
    lat(store_ind)                      = GPS_TimeTable.Data(gps_ind,1);
    lon(store_ind)                      = GPS_TimeTable.Data(gps_ind,2);
    alt(store_ind)                      = GPS_TimeTable.Data(gps_ind,3);
    pitch(store_ind)                    = GPS_TimeTable.Data(gps_ind,4);
    roll(store_ind)                     = GPS_TimeTable.Data(gps_ind,5);
    track(store_ind)                    = GPS_TimeTable.Data(gps_ind,6);
   
    % Setting Local Coords for the first thing in the list        
    if store_ind == 1
    
        % Starting point
        lat_start                           = double(lat(store_ind));
        lon_start                           = double(lon(store_ind));
        alt_start                           = double(alt(store_ind));
        
        % Pseudo starting point for the posision check: The first point
        % could probably be eliminated 
        lat_post(store_ind)                 = double(lat(store_ind));
        lon_post(store_ind)                 = double(lon(store_ind));
        alt_post(store_ind)                 = double(alt(store_ind));
        
    else
        
        % Grabbing the distance in meters from the origin using a built-in
        % matlab function that converts lat lon and alt to dx dy dz.
        [dx_from_origin(store_ind), dy_from_origin(store_ind), dz_from_origin(store_ind)] = geodetic2ned(lat(store_ind), lon(store_ind), alt(store_ind), lat_start, lon_start, alt_start, wgs84);
        
        % Temporary code to test flipping variables
%         dx_from_origin(store_ind) = dx_from_origin(store_ind);
%         dy_from_origin(store_ind) = dy_from_origin(store_ind);
%         dz_from_origin(store_ind) = dz_from_origin(store_ind);
        
        % Calculating distance from origin
        dist(store_ind)                     = sqrt(dx_from_origin(store_ind)^2 + dy_from_origin(store_ind)^2 + dz_from_origin(store_ind)^2);      

        % Calculating the change in distance from origin
        d_vect(store_ind)                   = dist(store_ind) - dist(store_ind-1);
        
        % Calculating time differences between the current and previous gps
        % and lidar time stamps
        duration_gps(store_ind)             = gps_closest_time(store_ind) - gps_closest_time(store_ind-1);
        duration_lidar(store_ind)           = lidar_time_stamp(store_ind) - lidar_time_stamp(store_ind-1);
        
        % Calculating speed
        speed_gps(store_ind)                = d_vect(store_ind) / duration_gps(store_ind);
        
        % Verification of the gps conversion process: plugging in the
        % dx dy dz from the origin and obtaining the lat lon and alt
        [lat_post(store_ind), lon_post(store_ind), h_post(store_ind)] = ned2geodetic(dx_from_origin(store_ind), dy_from_origin(store_ind), dz_from_origin(store_ind), lat_start, lon_start, alt_start, wgs84);
        
    end
        
    % Finding the smallest time difference between the current lidar time
    % stamp and the any imu time stamp. The index with the smallest time is
    % selected
    [imu_time_diff(store_ind), imu_ind] = min(abs(IMU_TimeTable.Time(:) - LiDAR_TimeTable.Time(i)));
    imu_closest_time(store_ind)         = IMU_TimeTable.Time(imu_ind,:);
    
    % The gps index is increased by one to increase odds of getting a good
    % imu selection
    imu_ind                             = imu_ind + 1;
    
    % Grabbing the Quaternion from the data file (W X Y Z)
    quat_temp                           = [IMU_TimeTable.Data(imu_ind,:)];

    % Getting the quaternion object (W X Y Z)
    quat                                = quaternion(quat_temp(1), quat_temp(2), quat_temp(3), quat_temp(4));
    
    % Alternatives
%     rot_mat = quat2rotm(quat); euler = quat2eul(quat);

    % Setting offset
    ROTATION_UPDATE                     = rotx(pitch(store_ind)) * roty(roll(store_ind)) * rotz(track(store_ind) - PCD_ROT_OFFSET);

    
    %% De-bugging time stamps
%     fprintf("\n LiDAR Time stamp: %s \n GPS Time Stamp: %s \n GPS Time Diff: %s \n IMU Time Stamp: %s \n IMU Time Diff: %s \n", LiDAR_TimeTable.Time(i), gps_closest_time(i), gps_time_diff(i), imu_closest_time(i), imu_time_diff(i))

    %% Adjust point cloud

    % Step 1: Load pc
    % Step 2: Resolve LiDAR and GPS Reference Frame
    % Step 3: Add Orientation 
    % Step 4: Add GPS Offset
    
    %% Step 1 - Load PC
    % Loading point cloud into friendly format
    xyzi                                = [double(LiDAR_TimeTable.Data(i).Location) double(LiDAR_TimeTable.Data(i).Intensity)];
    
    % Removing nans and infs from the array
    xyzi                                = xyzi( ~any( isnan(xyzi) | isinf(xyzi), 2),:);
    
    % Swapping X and Y axis
    xyzi                                = [xyzi(:,2), -xyzi(:,1), xyzi(:,3) xyzi(:,4)];
    
    %% Step 2 - Resolve LiDAR and GPS Reference Frame
    
    % Adding the offset values [Y X Z]
    xyzi(:,1)                           = xyzi(:,1) + gps_lidar_diff(1);
    xyzi(:,2)                           = xyzi(:,2) + gps_lidar_diff(2);
    xyzi(:,3)                           = xyzi(:,3) + gps_lidar_diff(3);
    
    %% Step 3 - Add Orientation
    
    % Applying the rotation
    xyzi(:,1:3)                         = [xyzi(:,1), xyzi(:,2), xyzi(:,3)] * ROTATION_UPDATE;

    %% Step 4 - Add GPS Offset
        
    % Adding GPS offset
    xyzi(:,1)                           = (xyzi(:,1)) + dx_from_origin(store_ind);
    xyzi(:,2)                           = (xyzi(:,2)) + dy_from_origin(store_ind);
    xyzi(:,3)                           = (xyzi(:,3)) + alt(store_ind);
    
    % Storing dx, dy for debugging
    dxy_store = [dxy_store; dx_from_origin(store_ind) dy_from_origin(store_ind) alt(store_ind)];
    
    %% Apphend to overall point cloud
    


    XYZI_Apphend{store_ind}             = xyzi;

    %% Progress bar

    % Updateing progress bar
    waitbar(store_ind/(loop_array(end)),f,sprintf('%1.1f',(store_ind/(loop_array(end))*100)))

    %% Clearing vars for safety
    
    clear quat
    
    %% Increasing the counter of the storage index
    
    if i ~= loop_array(end)
        store_ind = store_ind + 1;
    end

end

% Closing the progress bar
close(f)

% Generating plot array
plot_array = 1:1:store_ind;

disp('Processing complete, exporting array to point cloud object...')


disp('Point cloud object created. Saving to .pcd...')

%     FileName = string(export_dir) + "/Export_Cloud.pcd";

disp('Point cloud saved to .pcd! Plotting figures...')

%% Scatter plot
% color_array = [1 0 0 ; 0 1 0 ; 0 0 1 ; 0 1 1 ; 1 0 1 ; 1 1 0 ; 0 0 0];
% 
% j = 1;
% 
% figure;
% 
% hold all
% 
% for i = plot_array
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
%     origin_plt = scatter3(dx_from_origin(i), dy_from_origin(i), alt(i));
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
% 
% 
% end
% 
% hold off

% legend

%% PCD

% Initilizing the big xyzi array
Export_XYZI = [];

figure;

for i = plot_array
    
    Export_XYZI = [Export_XYZI; XYZI_Apphend{i}(:,1) XYZI_Apphend{i}(:,2) XYZI_Apphend{i}(:,3) XYZI_Apphend{i}(:,4)];
    
end

hold all
Export_Cloud    = pointCloud([Export_XYZI(:,1) Export_XYZI(:,2) Export_XYZI(:,3)], 'Intensity', Export_XYZI(:,4));
plot3(dxy_store(:,1),dxy_store(:,2),dxy_store(:,3),'--','Color','c','LineWidth',6)
scatter3(dxy_store(:,1),dxy_store(:,2),dxy_store(:,3),'o','MarkerEdgeColor','yellow','LineWidth',3)
scatter3(dxy_store(1,1),dxy_store(1,2),dxy_store(1,3),420,'*','MarkerFaceColor','green', 'MarkerEdgeColor','green','LineWidth',3)
scatter3(dxy_store(end,1),dxy_store(end,2),dxy_store(end,3),420,'*','MarkerFaceColor','red','MarkerEdgeColor','red','LineWidth',3)
pcshow(Export_Cloud)
view([0 0 90])
xlabel('x')
ylabel('y')
zlabel('z')
hold off

%% Step-by-step visulaizer for verification


if loop_bool
    
    figure; 

    hold all

    for i = 1:1:store_ind

            if size(XYZI_Apphend{i}) ~= [0,0]

                loop_cloud = pointCloud([XYZI_Apphend{i}(:,1) XYZI_Apphend{i}(:,2) XYZI_Apphend{i}(:,3)], 'Intensity', XYZI_Apphend{i}(:,4));

    %             Export_Cloud    = pointCloud([Export_XYZI(i,1) Export_XYZI(i,2) Export_XYZI(i,3)], 'Intensity', Export_XYZI(:,4));
                % plot3(dxy_store(:,1),dxy_store(:,2),dxy_store(:,3),'--','Color','c','LineWidth',6)
                scatter3(dxy_store(i,1),dxy_store(i,2),dxy_store(i,3))
                scatter3(dxy_store(1,1),dxy_store(1,2),dxy_store(1,3),420,'*','MarkerFaceColor','green')
                scatter3(dxy_store(end,1),dxy_store(end,2),dxy_store(end,3),420,'*','MarkerFaceColor','red')
                pcshow(loop_cloud)

            end

            pause

    end

    hold off

end

%% Making the plots prettier

figure
tiledlayout(2,2);

nexttile
plot(plot_array,gps_time_diff,'.-')
xlabel('Point Cloud Number')
ylabel('Time (s)')
title('Time stamp difference')

nexttile
plot(plot_array,dist,'.-')
xlabel('Point Cloud Number')
ylabel('Dist (m)')
title('Distance Traveled')

nexttile
plot(plot_array,duration_gps,'ro-')
hold on
plot(plot_array,duration_lidar,'b.-')
xlabel('Point Cloud Number')
ylabel('Time Stamp (s)')
legend('GPS','LiDAR')
title('DT Check')
hold off

nexttile
plot(plot_array,speed,'.-')
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
scatter(dy_from_origin,dx_from_origin, 'LineWidth', 10)
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

%% Misc Plots


figure
tiledlayout(2,2);

nexttile
plot(gps_time_diff)
xlabel('Point Cloud Number')
ylabel('Time (s)')
title('Time stamp difference')

nexttile
plot(dist)
xlabel('Point Cloud Number')
ylabel('Dist (m)')
title('Distance Traveled')

nexttile
plot(duration_gps,'r.-')
hold on
plot(duration_lidar,'b.-')
xlabel('Point Cloud Number')
ylabel('Time Stamp (s)')
legend('GPS','LiDAR')
title('DT Check')
hold off

nexttile
plot(speed_gps)
xlabel('Point Cloud Number')
ylabel('Speed (m/s)')
title('Speed')


figure
tiledlayout(1,3);

nexttile
geoplot(lat,lon,'.','LineWidth', 3)

nexttile
scatter(dy_from_origin,dx_from_origin, '.', 'LineWidth', 3)
axis equal

nexttile
geoplot(lat_post, lon_post, '.', 'LineWidth', 3)


disp('End program.')

