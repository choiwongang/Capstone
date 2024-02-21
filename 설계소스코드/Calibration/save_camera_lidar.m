clc; close all; clear;
%% Connect to webcam and lidar
lidar = velodynelidar("VLP16", "Port", 2368);
wcam = webcam(2);

v = read(lidar);
frame = snapshot(wcam);
ptCloudArray = reshape(v.Location,[],3);

% pointCloud 객체로 변환
ptCloud = pointCloud(ptCloudArray);

pcshow(ptCloud);
imshow(frame);
pcwrite(ptCloud, 'pointCloud_car_0m.pcd');
imwrite(frame, 'pointCloud_photo_0m.jpg');
