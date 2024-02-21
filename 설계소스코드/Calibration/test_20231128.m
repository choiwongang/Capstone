clc; close all; clear;
lidar = velodynelidar("VLP16", "Port", 2368);
wcam = webcam(2);
% 데이터 저장 설정
numFrames = 20; % 저장할 프레임 수
savePath = "C:\cameralidar_231128\block";

% 프레임별 데이터 저장
for i = 1:numFrames
    % 라이다 데이터 읽기
    v = read(lidar);
    frame = snapshot(wcam);
    lidar_data = reshape(v.Location,[],3);
    pointCloudData = pointCloud(lidar_data);
    % 파일로 저장
    filename1 = fullfile(savePath, sprintf('lidarblock_%d.pcd', i));
    pcwrite(pointCloudData, filename1);
    filename2 = fullfile(savePath, sprintf('camblock_%d.jpg', i));
    imwrite(frame, filename2);
    % 데이터 수집 간격 조절 (선택 사항)
    pause(0.1); % 0.1초 간격
end
% 재생할 데이터 파일 목록 불러오기
fileList = dir(fullfile(savePath, '*.pcd'));
numFiles = length(fileList);