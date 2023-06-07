%==========================================================================
%                            Travis Moleski
%
%                     FILE CREATION DATE: 10/11/2022
%
%                             lidar2Geo.m
%
% This program looks at a ROSBAG generated by the Ohio University van and
% creates a point cloud
%
%==========================================================================

%% Clear & Setup Workspace
clc; 
clear; 
close all;
format compact


%% Options

% minimum/maximum distance from LiDAR point of origin to include/exclude
use_dist_bool               = 0;
min_dist                    = 2;
max_dist                    = 20;

% Include which rings in final map
use_ring_bool               = 1;
% ring_min                    = 20;
% ring_max                    = 20;
ring_min                    = 15; % higher pointed lazer - max 31
ring_max                    = 0; % lower pointed lazer - min 0


%% Variable Initiation

LiDAR_Ref_Frame             = [0; 1.584; 1.444];
IMU_Ref_Frame               = [0; 0.336; -0.046];

% Correction frame:         LiDAR_Ref_Frame - IMU_Ref_Frame [Y X Z]
gps_to_lidar_diff           = [(LiDAR_Ref_Frame(1) - IMU_Ref_Frame(1)), (LiDAR_Ref_Frame(2) - IMU_Ref_Frame(2)), (LiDAR_Ref_Frame(3) - IMU_Ref_Frame(3))]; 

% distance traveled
gps_dist_traveled = [];

% filename init
time_now        = datetime("now","Format","uuuuMMddhhmmss");
time_now        = datestr(time_now,'yyyyMMddhhmmss');
file_p1         = 'point_cloud_source_' + string(time_now) + '.bag';

%% Loading the ROSBAG

% Location of rosbag
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_Simms/2022-10-11-09-24-00.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_Simms/2022-10-11-09-24-00.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_Simms/2022-10-11-09-28-18.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_Simms/2022-10-11-09-29-34.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_Simms/2022-10-11-09-31-55.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_Simms/2022-10-11-09-31-55.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_Simms/2022-10-11-09-33-39.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/Coach_Sturbois_Shortened/2022-10-14-14-31-07.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/Coach_Sturbois_Shortened/2022-10-14-14-31-42.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/Armitage_Shortened_Bags/2022-10-20-10-14-05_GRAV.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/Coach_Sturbois_Shortened/sturbois_straight_1.bag';
% file = '/media/autobuntu/chonk/chonk/git_repos/test_rosbag_reader/02_21_2023_ridges_outer_loop.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/gravel_lot/2023-03-09-14-46-23.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/Armitage_Shortened_Bags/2022-10-20-10-16-43_CHIP.bag'
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/gravel_lot/2023-03-09-14-46-23.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/03_13_2023_shortened_coach_sturbois/2023-03-13-11-01-55.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/gravel_lot/2023-03-14-13-12-31.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/blue_short/2023-03-15-14-09-13.bag'
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/03_13_2023_shortened_coach_sturbois/2023-03-13-10-56-38.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/03_23_23_SHORTENED_CAOCH/angel_ridge_1.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_bags_03_23_23/Mulligan_Road_2.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_bags_03_23_23/mulligan_road.bag';
file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/redmen/drive_by/rm_db_4.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/2023-05-24-16-31-32.bag'

% [file_p1, file_p2, ~] = uigetfile('*.bag','Get Bag');

% file = string(file_p2) + string(file_p1)

% Load the rosbag into the workspace
bag =  rosbag(file);

% Topics
topics = bag.AvailableTopics;
          
lidar_topic = select(bag,'Topic','velodyne_points');
lidar_msgs = readMessages(lidar_topic, 'DataFormat', 'struct');

ouster_topic = select(bag,'Topic','/ouster/points');
ouster_msgs = readMessages(ouster_topic, 'DataFormat','struct');

gps_topic = select(bag,'Topic','/gps/gps');
gps_msgs = readMessages(gps_topic, 'DataFormat', 'struct');


%% More Var Init

% cloud_break                 = 75;
cloud_break                 = length(lidar_msgs);
gps_pos_store               = zeros(cloud_break,3);
lidar_pos_store             = gps_pos_store;

time_store = [];

%% Timestamps

% Matching timestamps
[indexes, fromTimes, toTimes, diffs] = matchTimestamps(lidar_msgs, gps_msgs);

%Find which GPS message matches the first scan
matchedLidar                = lidar_msgs{1};
matchedGps_init             = gps_msgs{indexes(1)};


%% Initilizing the starting point
% Select reference point as first GPS reading (local)
origin = [matchedGps_init.Latitude, matchedGps_init.Longitude, matchedGps_init.Altitude];
% [xEast_init, yNorth_init, zUp_init] = latlon2local(matchedGps_init.Latitude, matchedGps_init.Longitude, matchedGps_init.Altitude, origin);


%%  CONVERT TO LIDAR FRAME:

% Getting the initial roll, pitch, and yaw values
% roll                        = matchedGps_init.Roll;
% pitch                       = matchedGps_init.Pitch;
% yaw                         = matchedGps_init.Track + 180;

% Setting the offset from the gps orientation to the lidar
% takes gps emu to local frame of lidar
gps2lidar = [ cosd(90) sind(90) 0;
             -sind(90) cosd(90) 0;
             0       0          1]; 
         
% Setting the (gps_to_lidar_diff) offset from the lidar offset to the gps
LidarOffset2gps = [ cosd(90) -sind(90)  0;
              sind(90)  cosd(90)   0;
              0        0           1]; 

% Setting initial orientation offset
% init_rotate_offset          = rotz(90-yaw)*roty(roll)*rotx(pitch);

% Offset the gps coord by the current orientation (in this case, initial) 
% Converts the ground truth to lidar frame
% groundTruthTrajectory       = [xEast_init, yNorth_init, zUp_init] * gps2lidar;

% Setting the updated difference between the lidar and gps coordiate and
% orientation
% Converts the offsett to the lidar frame
% gps_to_lidar_diff_update    = gps_to_lidar_diff * LidarOffset2gps * init_rotate_offset;

% Rotating the offset and adding them together
% LidarxEast_init             = groundTruthTrajectory(1) + gps_to_lidar_diff_update(1);
% LidaryNorth_init            = groundTruthTrajectory(2) + gps_to_lidar_diff_update(2);
% LidarzUp_init               = groundTruthTrajectory(3) + gps_to_lidar_diff_update(3);

% Making the vector of ^^^
% lidarTrajectory             = [LidarxEast_init,LidaryNorth_init,LidarzUp_init];

% Initilizing the lists
% gps_pos_store(1,:)          = groundTruthTrajectory;
% lidar_pos_store(1,:)        = lidarTrajectory;


%% Doing the data

fprintf('Max time delta is %f sec \n',max(abs(diffs)));

h = waitbar(0, "Cartographing...");
for cloud = 1:length(lidar_msgs)
    
    tStart = tic;
    
    % Reading the current point cloud and matched gps coord
    current_cloud               = lidar_msgs{cloud};
    matched_stamp               = gps_msgs{indexes(cloud)};
    
    % Converting the gps coord to xyz (m)
    [xEast, yNorth, zUp]        = latlon2local(matched_stamp.Latitude, matched_stamp.Longitude, matched_stamp.Altitude, origin);
    
    % Grabbing the angles
    roll                        = matched_stamp.Roll;
    pitch                       = matched_stamp.Pitch;
    yaw                         = matched_stamp.Track+180;
    
    % Creating the rotation matrix
    rotate_update               = rotz(90-yaw)*roty(roll)*rotx(pitch);
     
    % Offset the gps coord by the current orientation (in this case, initial) 
    % Converts the ground truth to lidar frame
    groundTruthTrajectory       = [xEast, yNorth, zUp] * gps2lidar ;
    
    % Setting the updated difference between the lidar and gps coordiate and
    % orientation
    % Converts the offsett to the lidar frame
    gps_to_lidar_diff_update    = gps_to_lidar_diff * LidarOffset2gps * rotate_update;

    % Offset the gps coord by the current orientation (in this case, initial) 
    % Converts the ground truth to lidar frame
    groundTruthTrajectory       = groundTruthTrajectory;
    
    % Rotating the offset and adding them together
    LidarxEast                  = groundTruthTrajectory(1)  + gps_to_lidar_diff_update(1);
    LidaryNorth                 = groundTruthTrajectory(2)  + gps_to_lidar_diff_update(2);
    LidarzUp                    = groundTruthTrajectory(3)  + gps_to_lidar_diff_update(3);
    
    % Making the vector of ^^^
    lidarTrajectory             = [LidarxEast, LidaryNorth, LidarzUp];
    
    % Reading the current cloud for xyz, intensity, and ring (channel) values
    xyz_cloud                   = rosReadXYZ(current_cloud);
    intensities                 = rosReadField(current_cloud, 'intensity');
    ring                        = rosReadField(current_cloud, 'ring');
    xyz_cloud(:,4)              = intensities;
    xyz_cloud(:,5)              = ring;
    
    % Here are options for trimming data ~~~~~~~
    
    if use_ring_bool
        
        % Eliminate points based on channel
        xyz_cloud(xyz_cloud(:,5) < ring_max, :) = [];
        xyz_cloud(xyz_cloud(:,5) > ring_min, :) = [];

    end
    
    if use_dist_bool
        
        % Eliminate points based on distance
        xyz_cloud(sqrt(xyz_cloud(:,1).^2 + xyz_cloud(:,2).^2 + xyz_cloud(:,3).^2) <= min_dist, :) = [];
        xyz_cloud(sqrt(xyz_cloud(:,1).^2 + xyz_cloud(:,2).^2 + xyz_cloud(:,3).^2) >= max_dist, :) = [];

    end
    
    % Eliminiating infs and nans from the xyz data - there may be a way to
    % use the 
    xyz_cloud                   = xyz_cloud( ~any( isnan(xyz_cloud) | isinf(xyz_cloud), 2),:);
    
    % EXPERIMENT sort rows
    xyz_cloud = sortrows(xyz_cloud,5);
    
    % Transforming the point cloud
    tform                       = rigid3d(rotate_update, [lidarTrajectory(1) lidarTrajectory(2) lidarTrajectory(3)]);
    
    % Creating the point cloud object3
    pointClouXYZI_curr          = pointCloud([xyz_cloud(:,1), xyz_cloud(:,2), xyz_cloud(:,3)], 'Intensity',  xyz_cloud(:,4));
    pointClouXYZI_curr          = pctransform(pointClouXYZI_curr, tform);
   
    gps_pos_store(cloud,:)      = groundTruthTrajectory;
    lidar_pos_store(cloud,:)    = lidarTrajectory;
    
    pointCloudList{cloud}       = pointClouXYZI_curr;
    
%     mergeGridStep = 0.1;
%     pointCloudList = pcmerge(pointCloudList, pointClouXYZI_curr, mergeGridStep);
    
    if cloud > cloud_break
        break
    end
    
    %% Time to Completion Estimation
    tEnd = toc(tStart);
    time_store = [time_store; tEnd];
    time_avg = mean(time_store);
    est_time_to_complete = (time_avg * (cloud_break - cloud));
    
    %% Waitbar
    
    waitbar(cloud/cloud_break,h,sprintf('Cloud %d out of %d, ~ %0.1f sec left',cloud, cloud_break, est_time_to_complete))
    
end

delete(h)


%% Compiling the map

disp("Making the map, sire...")
pointCloudList = pccat([pointCloudList{:}]);


%% Displaying the map

figure

hold on

% Plotting the line between the lidar and gps
for point = 1:length(lidar_pos_store)
    
    plot3([lidar_pos_store(point,1) gps_pos_store(point,1)],...
          [lidar_pos_store(point,2) gps_pos_store(point,2)],...
          [lidar_pos_store(point,3) gps_pos_store(point,3)],...
          'linewidth',3)
      
end

% Plotting the lidar and gps points
scatter3(gps_pos_store(1,1),gps_pos_store(1,2),gps_pos_store(1,3),420,'^','MarkerFaceColor','yellow')
scatter3(gps_pos_store(end,1),gps_pos_store(end,2),gps_pos_store(end,3),420,'^','MarkerFaceColor','blue')
scatter3(gps_pos_store(:,1),gps_pos_store(:,2),gps_pos_store(:,3),50,'^','MarkerFaceColor','magenta')
scatter3(lidar_pos_store(:,1),lidar_pos_store(:,2),lidar_pos_store(:,3),50,'^','MarkerFaceColor','cyan')

% Plotting the point cloud
pcshow(pointCloudList);

view([0 0 90])

%% Length Traveled

for gps_pos_store_idx = 2:1:length(gps_pos_store)
    
    % Calculate between two points
    x_component = (gps_pos_store(gps_pos_store_idx,1) - gps_pos_store(gps_pos_store_idx-1,1))^2;
    y_component = (gps_pos_store(gps_pos_store_idx,2) - gps_pos_store(gps_pos_store_idx-1,2))^2;
    z_component = (gps_pos_store(gps_pos_store_idx,3) - gps_pos_store(gps_pos_store_idx-1,3))^2;
    dist_traveled_temp = (x_component + y_component + z_component)^(1/2);
    
    % Apphend
    gps_dist_traveled = [gps_dist_traveled; dist_traveled_temp];
    
end

tot_dist_traveled = sum(gps_dist_traveled) / 0.3048; % freedom units best units


%% Save the PCD

save_ans = questdlg('Save pcd?', 'Save pcd?', 'Yes', 'No', 'No');

switch save_ans
    
    case 'Yes'
        
        name_ans        = inputdlg({'Enter Filename:'}, 'Filename', [1 35], {'pcd.pcd'});
%         name_ans        = name_ans{:};
%         name_ans = name_ans + "_FROM_FILE_" + string(file_p1(1:end-4)) + ".pcd";
                
        export_dir      = uigetdir();
        PCDFileName     = string(fullfile(export_dir, name_ans));
        
        pcwrite(pointCloudList,PCDFileName)
        
        disp('File Saved! :D')
        
    case 'No'
        
        warning('WILL NOT SAVE THE PCD!')
        
end


%% End Program 

% web('https://www.youtube.com/watch?v=DPBvMsT3prg&ab_channel=AdesRizaTV')

disp('End Program!')

