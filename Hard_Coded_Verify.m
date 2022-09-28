% This program takes hard coded data that was made in the
% LiDAR_GPS_IMU_Timetable_Maker function and verifies stuff for
% verification purposes

% Clearing Workspace
clear all; close all; clc;

% Location of Rosbag File - uncomment if you need to choose the file
% [bag_file, bag_path] = uigetfile('.bag', 'Select Rosbag');

% Hard coding the path
bag_path = '/media/autobuntu/chonk/chonk/DATA/chonk_ROSBAG/shortened_Simms/2022-09-20-10-58-09.bag';

%Initializing rosbag
bag_init            = rosbag(bag_path);

% Variable init
Orientation = [];
Lat_Lon_Alt_Track = [];

%% Handling GPS Loading

% Topics
gps_topic       = '/gps/gps';

% Selecting the bag
gps_bag         = select(bag_init, 'Topic', gps_topic);

% Creating Structure
gps_struct      = readMessages(gps_bag,'DataFormat','struct');


%% Handling IMU Loading

% Topics
gps_imu_topic	= '/gps/imu';

% Selecting the bag
gps_imu_bag         = select(bag_init, 'Topic', gps_imu_topic);

% Creating Structure
gps_imu_struct      = readMessages(gps_imu_bag,'DataFormat','struct');

%% Handling LiDAR Loading

% RPM of the LiDAR    
RPM                                             = 900;

% Device Model (string): VLP16 VLP32C HDL32E HDL64E VLS128
device_model                                    = "VLP32C";

% Number of channels
num_channels                                    = 32;

% Converting the RPM into Hz then finding dT for each revolution. This
% will hopefully make a point cloud with one full revolution.
dT                      = 1 / (RPM / 60);

% Velodyne Topic
velodyne_packets_topic	= '/velodyne_packets';

% Selecting the bag
velodyne_packets_bag    = select(bag_init, 'Topic', velodyne_packets_topic);

% Creating Structure
velodyne_packets_struct = readMessages(velodyne_packets_bag,'DataFormat','struct');

disp('Structure Creation Completed.')

% Allocating memory for the matrices: Needs length of each sweep.

% Reading the velodyne stuffs
veloReader_packets      = velodyneROSMessageReader(velodyne_packets_struct,device_model);

% Extracting Point Clouds
timeDuration_packets    = veloReader_packets.StartTime;

% Read first point cloud recorded
ptCloudObj_packets      = readFrame(veloReader_packets, timeDuration_packets);

% Access Location Data
ptCloudLoc_packets      = ptCloudObj_packets.Location;

%% Processing the point cloud data and exporting to a helpful format

clear Epoch


% Memory allocation

% Checking Length
memory_array_xyzi       = double(zeros(1, length(ptCloudLoc_packets(:,:,1)) * num_channels));
memory_array_pt_pack    = double(zeros(32, length(ptCloudLoc_packets(:,:,1)) * num_channels));
memory_array_XYZI_TOT   = double(zeros(length(velodyne_packets_struct),4));

% Allocation
x_append                = memory_array_xyzi;
y_append                = memory_array_xyzi;
z_append                = memory_array_xyzi;
int_append              = memory_array_xyzi;
ptCloudLoc_packets      = memory_array_pt_pack;
XYZI_TOT                = memory_array_XYZI_TOT;

% Loop: Exporting PCDs to a structure that has the time stamp and each
% associated point cloud
for i = 1:length(velodyne_packets_struct)

    dT_loop                 = dT * i;

    % Extracting Point Clouds
    timeDuration_packets    = veloReader_packets.StartTime + seconds(dT_loop);

    % Read first point cloud recorded
    ptCloudObj_packets      = readFrame(veloReader_packets, timeDuration_packets);

    % Access Location Data
    ptCloudLoc_packets      = ptCloudObj_packets.Location;

    % Access Intensity Data
    ptCloudInt_packets      = ptCloudObj_packets.Intensity;

    % Extracting data
    for j = 1:32

        x                       = ptCloudLoc_packets(j,:,1);
        y                       = ptCloudLoc_packets(j,:,2);
        z                       = ptCloudLoc_packets(j,:,3);
        int                     = ptCloudInt_packets(j,:);

        x_append                = [x_append x];
        y_append                = [y_append y];
        z_append                = [z_append z];
        int_append              = [int_append int];

    end % Extracting data

    XYZI_TOT                = [x_append' y_append' z_append'];

    pc                      = pointCloud(XYZI_TOT, 'Intensity', int_append');

    % Resetting the arrays
    x_append                = memory_array_xyzi;
    y_append                = memory_array_xyzi;
    z_append                = memory_array_xyzi;
    int_append              = memory_array_xyzi;

    % Get Timestamp
    Epoch           = double(velodyne_packets_struct{i, 1}.Packets(1).Stamp.Sec) + double(velodyne_packets_struct{i, 1}.Packets(1).Stamp.Nsec) * 10^-9;
%         Time            = datetime(Epoch,'ConvertFrom','posixtime','Format','HH:mm:ss.SSSS');  

    % Append to TimeTable
    LiDAR_TimeTable.Time(i,1) = Epoch';
    LiDAR_TimeTable.Data(i) = pc;

end  % Exporting PCD

%% Putting the GPS data into a easy structure

clear Epoch

for i = 1:length(gps_struct)
    
    % Getting Epoch Time
    Epoch(i)           = double(gps_struct{i}.Header.Stamp.Sec) + double(gps_struct{i}.Header.Stamp.Nsec) * 10^-9;
    
    % Getting the lat, lon, and alt and slamming those puppies into an
    % array for yah. Pitch Roll Yaw (called track for some reason...)
    Lat_Lon_Alt_Pitch_Roll_Track = [Lat_Lon_Alt_Track; gps_struct{i}.Latitude gps_struct{i}.Longitude gps_struct{i}.Altitude gps_struct{i}.Pitch gps_struct{i}.Roll gps_struct{i}.Track];
    
end

% Exporting the GPS timetable

% Create the Time Table
GPS_TimeTable.Time = Epoch';
GPS_TimeTable.Data = Lat_Lon_Alt_Track;

%% Putting the IMU data into an easy structure

clear Epoch
    
% Topics
gps_imu_topic	= '/gps/imu';

% Selecting the bag
gps_imu_bag         = select(bag_init, 'Topic', gps_imu_topic);

% Creating Structure
gps_imu_struct      = readMessages(gps_imu_bag,'DataFormat','struct');

for i = 1:length(gps_imu_struct)

    % Get Timestamp
    Epoch(i)           = double(gps_imu_struct{i}.Header.Stamp.Sec) + double(gps_imu_struct{i}.Header.Stamp.Nsec) * 10^-9;
%         Date_Time(i)    = datetime(Epoch,'ConvertFrom','posixtime','Format','HH:mm:ss.SSSS');

    % Get Orientation
    Orientation     = [Orientation; gps_imu_struct{i, 1}.Orientation.W gps_imu_struct{i, 1}.Orientation.X gps_imu_struct{i, 1}.Orientation.Y gps_imu_struct{i, 1}.Orientation.Z];

end

% Exporting the IMU timetable

% Create the Time Table
IMU_TimeTable.Time = Epoch';
IMU_TimeTable.Data = Orientation;

%% Loading the 20th point cloud

% Loading the Point Cloud
pc_20 = LiDAR_TimeTable.Data(20);

% Loading the Time Stamp (Epoch)
pc_ts_20 = LiDAR_TimeTable.Time(20);

%% Guessing the closest GPS

[gps_time_diff,gps_ind]     = min(abs(GPS_TimeTable.Time(:) - pc_ts_20));
    
gps_ind = gps_ind + 1;

gps_closest_time            = GPS_TimeTable.Time(gps_ind);

fprintf("\nLiDAR Time = %1.f \nGPS Time = %1.f \nDifference = %0.30f\n\n", pc_ts_20*10^9, gps_closest_time*10^9, gps_time_diff)


















