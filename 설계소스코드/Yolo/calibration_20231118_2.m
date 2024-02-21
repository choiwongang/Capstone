clc; close all; clear; 
lidar = velodynelidar("VLP16", "Port", 2368);
v = read(lidar);
pcshow(v);
title('pointCloud');
xlabel('x');
ylabel('y');
zlabel('z');