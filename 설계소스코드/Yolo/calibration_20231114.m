clc; close all; clear;
lidar = velodynelidar("VLP16","Port",2368);
while true
    finalBox = Live_Webcam_YOLOv2();
    disp(finalBox);
end