ptCloud = pcread('pointCloud_car_13m.pcd');
% pcshow(ptCloud);
% xlabel('X-axis Label'); % X축 레이블 설정
% ylabel('Y-axis Label'); % Y축 레이블 설정
%% Camera Matrix calibration 
Mc = [ 827.4589 0.000000 325.2333 0.000000;
       0.000000 650.7102 214.4634 0.000000;
       0.000000 0.000000 1.000000 0.000000];
%% Camera Matrix calibration 
Mc1 = [ 827.4589 0.000000 325.2333;
       0.000000 650.7102 214.4634;
       0.000000 0.000000 1.000000];

% %% Matrix rotation Lidar-camera
Rlc = [1 0.17 0;
       0 -0.03 -1;
       0 -1 0];    

%% Matrix translation Lidar-camera
Tlc = [0;
       0;
      0.3];


%% Matrix rotation translation  
R_T_lc = [[Rlc Tlc]; 0 0 0 1];
% % % 이미지 불러오기
image = imread('pointCloud_photo_13m.jpg');
% 
% % % 이미지 크기 얻기
[rows, cols, ~] = size(image);

% 각 픽셀의 위치 및 색상 정보 추출
[x, y] = meshgrid(1:cols, 1:rows);
x = x(:);
y = y(:);
colors = double(reshape(image, [], 3)) / 255; % 색상 데이터를 double 형식으로 변환

% 2D 플롯에서 픽셀을 점으로 표시
figure;
scatter(x, y, 1, colors, 'filled');
axis ij; % y축을 이미지 좌표계와 일치시킴
axis equal; % 축의 스케일을 같게 설정
hold on;

data = ptCloud.Location;
x_data = data(:,1);
y_data = data(:,2);
z_data = data(:,3);
validIdx = y_data >= 0;
x_data = x_data(validIdx);
y_data = y_data(validIdx);
z_data = z_data(validIdx);

 %% Transform Lidar data to image data
lidar_data = [-z_data'; -y_data'; -x_data'; ones(size(x_data))'];
trans = Mc * R_T_lc * lidar_data;
py = round(trans(1,:) ./ trans(3,:)); 
px = round(trans(2,:) ./ trans(3,:)); 
% title('Point Cloud'); % 타이틀 설정
% xlabel('X-axis Label'); % X축 레이블 설정
% ylabel('Y-axis Label'); % Y축 레이블 설정
% x_data = x_data*40.93788 + 275.39708;
% z_data = z_data*(-87.13639)+ 190.61856;
% validIdx = x_data > 250 & x_data < 503;
% x_data = x_data(validIdx);
% y_data = y_data(validIdx);
% z_data = z_data(validIdx);
% validIdx = z_data > 200 & z_data < 400;
% x_data = x_data(validIdx);
% y_data = y_data(validIdx);
% z_data = z_data(validIdx);

% 포인트 클라우드 플롯
% imshow(image);
% title('Photo'); % 타이틀 설정
plot(px,py, '.','Color', 'blue');
xlim([0,640]);
ylim([0,480]);
title('Point Cloud'); % 타이틀 설정
xlabel('X-axis Label'); % X축 레이블 설정
ylabel('Y-axis Label'); % Y축 레이블 설정
hold off;
% 이미지 표시
