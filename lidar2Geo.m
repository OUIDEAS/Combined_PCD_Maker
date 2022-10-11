clc;clear;close all
format compact

LiDAR_Ref_Frame             = [0; 1.584; 1.444];
IMU_Ref_Frame               = [0; 0.336; -0.046];
% Correction frame:         LiDAR_Ref_Frame - IMU_Ref_Frame [Y X Z]
gps_to_lidar_diff  = [(LiDAR_Ref_Frame(1) - IMU_Ref_Frame(1)), ...
                      (LiDAR_Ref_Frame(2) - IMU_Ref_Frame(2)), ...
                      (LiDAR_Ref_Frame(3) - IMU_Ref_Frame(3))]; 

% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_Simms/2022-10-11-09-24-00.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_Simms/2022-10-11-09-28-18.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_Simms/2022-10-11-09-29-34.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_Simms/2022-10-11-09-31-55.bag';
% file = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_Simms/2022-10-11-09-33-39.bag';
% file = '/media/travis/moleski/ROSBAGS_WILHELM/trimmed/2022-10-07-12-25-42.bag';
% file = '/media/travis/moleski/ROSBAGS_WILHELM/trimmed/2022-10-07-14-36-02.bag';
file = '/media/travis/moleski/FOR_RHETT/2022-09-20-10-58-09.bag';
% file = '/media/travis/moleski/ROSBAGS_WILHELM/2022-03-01-14-13-06.bag';

bag =  rosbag(file);
topics = bag.AvailableTopics;

lidar_topic = select(bag,'Topic','velodyne_points');
lidar_msgs = readMessages(lidar_topic, 'DataFormat', 'struct');

gps_topic = select(bag,'Topic','/gps/gps');
gps_msgs = readMessages(gps_topic, 'DataFormat', 'struct');

[indexes, fromTimes, toTimes, diffs] = matchTimestamps(lidar_msgs, gps_msgs);

% cloud_break = 5;
cloud_break = length(lidar_msgs);
gps_position_store      = zeros(cloud_break,3);
lidar_position_store    = gps_position_store;

%Find which GPS message matches the first scan (
matchedLidar = lidar_msgs{1};
matchedGps_init = gps_msgs{indexes(1)};

% Select reference point as first GPS reading
origin = [matchedGps_init.Latitude, matchedGps_init.Longitude, matchedGps_init.Altitude];
[xEast_init, yNorth_init, zUp_init] = latlon2local(matchedGps_init.Latitude, matchedGps_init.Longitude, matchedGps_init.Altitude, origin);

%  CONVERT TO LIDAR FRAME:

roll = matchedGps_init.Roll;
pitch = matchedGps_init.Pitch;
yaw = matchedGps_init.Track + 180;

gps2lidar = [ cosd(90) sind(90) 0;
             -sind(90) cosd(90) 0;
             0       0          1];       
LidarOffset2gps = [ cosd(90) -sind(90)  0;
              sind(90)  cosd(90)   0;
              0        0           1]; 

init_rotate_offset= rotz(90-yaw)*roty(roll)*rotx(pitch);
          
          
groundTruthTrajectory    = [xEast_init, yNorth_init, zUp_init] * gps2lidar;
gps_to_lidar_diff_update = gps_to_lidar_diff * LidarOffset2gps * init_rotate_offset;

LidarxEast_init =   groundTruthTrajectory(1)  + gps_to_lidar_diff_update(1);
LidaryNorth_init =  groundTruthTrajectory(2)  + gps_to_lidar_diff_update(2);
LidarzUp_init =     groundTruthTrajectory(3)  + gps_to_lidar_diff_update(3);

groundTruthTrajectory = groundTruthTrajectory;
lidarTrajectory = [LidarxEast_init,LidaryNorth_init,LidarzUp_init];

gps_position_store(1,:) = groundTruthTrajectory;
lidar_position_store(1,:) = lidarTrajectory;

init_cloud = rosReadXYZ(matchedLidar);
intensities = rosReadField(matchedLidar, 'intensity');
ring = rosReadField(matchedLidar, 'ring');
init_cloud(:,4) = intensities;
init_cloud(:,5) = ring;

tform     = rigid3d(init_rotate_offset, [lidarTrajectory(1) lidarTrajectory(2) lidarTrajectory(3)]);
init_pcl = pointCloud([init_cloud(:,1), init_cloud(:,2), init_cloud(:,3)], 'Intensity',  init_cloud(:,4));
init_pcl = pctransform(init_pcl, tform);

pointCloudList{1}  = init_pcl; 
fprintf('Max time delta is %f sec \n',max(abs(diffs)));

h = waitbar(0, "Cartographing...");
for cloud = 1:length(lidar_msgs)
    
    current_cloud = lidar_msgs{cloud};
    matched_stamp = gps_msgs{indexes(cloud)+1};
    
    [xEast, yNorth, zUp] = latlon2local(matched_stamp.Latitude, matched_stamp.Longitude, matched_stamp.Altitude, origin);
    
    roll  = matched_stamp.Roll;
    pitch = matched_stamp.Pitch;
    yaw   = matched_stamp.Track+180;
     
    rotate_update = rotz(90-yaw)*roty(roll)*rotx(pitch);
     
    groundTruthTrajectory    = [xEast, yNorth, zUp] * gps2lidar ;
    gps_to_lidar_diff_update = gps_to_lidar_diff * LidarOffset2gps * rotate_update;
    

    LidarxEast =   groundTruthTrajectory(1)  + gps_to_lidar_diff_update(1);
    LidaryNorth =  groundTruthTrajectory(2)  + gps_to_lidar_diff_update(2);
    LidarzUp =     groundTruthTrajectory(3)  + gps_to_lidar_diff_update(3);

    groundTruthTrajectory = groundTruthTrajectory;
    lidarTrajectory = [LidarxEast, LidaryNorth, LidarzUp];
    
    xyz_cloud     = rosReadXYZ(current_cloud);
    xyz_cloud = rosReadXYZ(current_cloud);
    intensities = rosReadField(current_cloud, 'intensity');
    ring = rosReadField(current_cloud, 'ring');
    xyz_cloud(:,4) = intensities;
    xyz_cloud(:,5) = ring;
    
    xyz_cloud = xyz_cloud( ~any( isnan(xyz_cloud) | isinf(xyz_cloud), 2),:);
    
    tform     = rigid3d(rotate_update, [lidarTrajectory(1) lidarTrajectory(2) lidarTrajectory(3)]);
    
    pointClouXYZI_curr = pointCloud([xyz_cloud(:,1), xyz_cloud(:,2), xyz_cloud(:,3)], 'Intensity',  xyz_cloud(:,4));
    pointClouXYZI_curr = pctransform(pointClouXYZI_curr, tform);
   
    gps_position_store(cloud,:) = groundTruthTrajectory;
    lidar_position_store(cloud,:) = lidarTrajectory;
    
    pointCloudList{cloud}  = pointClouXYZI_curr;
    
%     mergeGridStep = 0.1;
%     pointCloudList = pcmerge(pointCloudList, pointClouXYZI_curr, mergeGridStep);
    
    if cloud > cloud_break
        break
    end
    waitbar(cloud/cloud_break,h,sprintf('Cloud %0.1f out of %0.1f',cloud, cloud_break))
    
end
delete(h)

disp("Making the map, sire...")
pointCloudList = pccat([pointCloudList{:}]);
hold on
for point = 1:length(lidar_position_store)
    plot3([lidar_position_store(point,1) gps_position_store(point,1)],...
          [lidar_position_store(point,2) gps_position_store(point,2)],...
          [lidar_position_store(point,3) gps_position_store(point,3)],...
          'linewidth',3)
end
web('https://www.youtube.com/watch?v=DPBvMsT3prg&ab_channel=AdesRizaTV')
% axis equal
% grid on
scatter3(gps_position_store(1,1),gps_position_store(1,2),gps_position_store(1,3),420,'^','MarkerFaceColor','yellow')
scatter3(gps_position_store(end,1),gps_position_store(end,2),gps_position_store(end,3),420,'^','MarkerFaceColor','blue')
scatter3(gps_position_store(:,1),gps_position_store(:,2),gps_position_store(:,3),50,'^','MarkerFaceColor','magenta')
scatter3(lidar_position_store(:,1),lidar_position_store(:,2),lidar_position_store(:,3),50,'^','MarkerFaceColor','cyan')
pcshow(pointCloudList);
view([0 0 90])