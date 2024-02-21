% 재생할 데이터 파일 목록 불러오기
fileList = dir(fullfile(savePath, '*.pcd'));
numFiles = length(fileList);

% 포인트 클라우드 데이터 순차적 재생
for i = 1:numFiles
    % 파일로부터 포인트 클라우드 데이터 로드
    filename = fullfile(fileList(i).folder, fileList(i).name);
    pointCloudData = pcread(filename);

    % 포인트 클라우드 시각화
    pcshow(pointCloudData);
    drawnow;

    % 재생 간격 조절 (선택 사항)
    pause(0.1); % 0.1초 간격
end
