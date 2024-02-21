
%% Camera Matrix calibration 
Mc = [ 827.4589 0.000000 325.2333 0.000000;
       0.000000 650.7102 214.4634 0.000000;
       0.000000 0.000000 1.000000 0.000000];
%% Camera Matrix calibration 
Mc1 = [ 827.4589 0.000000 325.2333;
       0.000000 650.7102 214.4634;
       0.000000 0.000000 1.000000];

% %% Matrix rotation Lidar-camera
Rlc = [1 0 0;
       0 0 -1;
       0 1 0];    

%% Matrix translation Lidar-camera
Tlc = [0;
       0;
       0];


%% Matrix rotation translation  
R_T_lc = [[Rlc Tlc]; 0 0 0 1];


data = [-0.489 3.74 -0.33;
    0.45 3.69 -0.32
    -0.517 2.5 -0.69
    0.249 2.497 -0.57];
x_data = data(:,1);
y_data = data(:,2);
z_data = data(:,3);

 %% Transform Lidar data to image data
lidar_data = [-z_data'; y_data'; x_data'; ones(size(x_data))'];
trans = Mc * R_T_lc * lidar_data;
py = (trans(1,:) ./ trans(3,:)); 
px = (trans(2,:) ./ trans(3,:)); 
plot(px,py);