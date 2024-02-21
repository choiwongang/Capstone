function Live_Webcam_YOLOv2()
% Connect to webcam and lidar
lidar = velodynelidar("VLP16", "Port", 2368);
wcam = webcam(2);
wcam.Resolution = wcam.AvailableResolutions{1};
viewer = vision.DeployableVideoPlayer();

%% Camera Matrix calibration 
Mc = [ 827.4589 0.000000 325.2333 0.000000;
       0.000000 650.7102 214.4634 0.000000;
       0.000000 0.000000 1.000000 0.000000];
%% Camera Matrix calibration 
Mc1 = [ 827.4589 0.000000 325.2333;
       0.000000 650.7102 214.4634;
       0.000000 0.000000 1.000000];

%% Matrix rotation Lidar-camera
Rlc = [1 0.17 0;
       0 -0.03 -1;
       0 -1 0];    

%% Matrix translation Lidar-camera
Tlc = [0;
       0;
       0.3];
%% Matrix rotation translation  
R_T_lc = [[Rlc Tlc]; 0 0 0 1];
%% Auxiliary variables
fps = 0;
avgfps = [];
cont = true;
%% Load your Trained YOLOv2 detector
load yoloTargetCar.mat
figure 
hold on;
while cont
    v = read(lidar);
    frame = snapshot(wcam);
    %% Extract XYZ data
    lidar_data = reshape(v.Location,[],3);
    x_data = lidar_data(:,1);
    y_data = lidar_data(:,2);
    z_data = lidar_data(:,3);
    %% Limits of data
    validIdx = y_data >= 0;
    x_data = x_data(validIdx);
    y_data = y_data(validIdx);
    z_data = z_data(validIdx);
    tic; 
    %% Count FPS
    sz = size(frame);
    resz =[128 128];
    frame1 = imresize(frame, resz);
    % detect object with trained yolo network
    tic;
    [bbox, score, label] = detect(detector, frame1, 'Threshold', 0.6, 'ExecutionEnvironment', "cpu");
%   You can use below after generating CUDA mex with GPU Coder.
%   [bbox, score, label] = yolov2_detect_mex(frame1, 0.55);
    newt = toc;
    % fps
    fps = .9*fps + .1*(1/newt);
    avgfps = [avgfps, fps];
    bbox(:,1) = bbox(:,1)*sz(2)/resz(2);
    bbox(:,2) = bbox(:,2)*sz(1)/resz(1);
    bbox(:,3) = bbox(:,3)*sz(2)/resz(2);
    bbox(:,4) = bbox(:,4)*sz(1)/resz(1);
    num = numel(bbox(:,1));
    detectedImg = frame;
    annotation = [];
    color =[];
    bbox1 =[];
    if num > 1 
        continue
    end
    if num == 1
        label = categorical(label);
        k=1;
        % set annotation and color of bbox
        for n=1:num
            if label(n) == 'TargetCar'
                annotation{k} = sprintf('%s: ( %f)', label(n), score(n));
                color{k} = 'yellow';
                bbox1(k,:) = bbox(n,:);
                k=k+1;
            end
        end
        %% Limit PointCloud only in range of the Bbox
        j = 1;
        for i = 1:length(x_data)    
            angh_l = abs(atan(y_data(i) / x_data(i)));
            if(angh_l > 1)
                z_arr(j) = z_data(i);
                y_arr(j) = y_data(i);
                x_arr(j) = x_data(i);
                j = j + 1;
            end
        end
        x_data = x_arr';
        y_data = y_arr';
        z_data = z_arr';
        subplot(1,2,2);
        plot3(x_data, y_data, z_data, '.')
        grid on
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('Point Cloud in angle camera range')
        axis equal; 
        %% Matrix lidar data referring to camera data
        lidar_data = [-z_data'; -y_data'; -x_data'; ones(size(x_data))'];

        %% Transform Lidar data to image data
        trans = Mc * R_T_lc * lidar_data;
        px = round(trans(1,:) ./ trans(3,:)); 
        py = round(trans(2,:) ./ trans(3,:)); 
        %% Filter based on px, py conditions pixel bounding need
        % valid_indices = ~(px < bbox(1) | px > bbox(3) | py < bbox(2) | py > bbox(4));
        % ax = px(valid_indices); 
        % by = py(valid_indices);
        % x_data = x_data(valid_indices);
        % y_data = y_data(valid_indices);
        % z_data = z_data(valid_indices);
        position = [px(:),py(:)];
        %% add anotations to show the result
        detectedImg = insertObjectAnnotation(detectedImg, 'rectangle', bbox1, annotation,'Color',color);   
        %% Color representation
        for i=1:length(y_data)
             if (y_data(i)<2)
                detectedImg = insertMarker(detectedImg, position, 'Color', 'white'); 
             elseif (y_data(i)<4 && y_data(i)>=2)
                detectedImg = insertMarker(detectedImg, position, 'Color', 'red'); 
             elseif (y_data(i)<6 && y_data(i)>=4)
                detectedImg = insertMarker(detectedImg, position, 'Color', 'yellow');  
             elseif (y_data(i)<8 && y_data(i)>=6)
                detectedImg = insertMarker(detectedImg, position, 'Color', 'green'); 
             elseif (y_data(i)<10 && y_data(i)>=8)
                detectedImg = insertMarker(detectedImg, position, 'Color', 'blue'); 
             elseif (y_data(i)<12 && y_data(i)>=10)
                detectedImg = insertMarker(detectedImg, position, 'Color', 'cyan');  
             elseif (y_data(i)<14 && y_data(i)>=12)
                detectedImg = insertMarker(detectedImg, position, 'Color', 'magenta'); 
             elseif (y_data(i)>=14)
                detectedImg = insertMarker(detectedImg, position, 'Color', 'b');  
             end
         end
    end
    % show the result
    detectedImg = insertText(detectedImg, [1, 1],  sprintf('FPS %2.2f', fps), 'FontSize', 26, 'BoxColor', 'y');    
   
    viewer(detectedImg)
    cont = isOpen(viewer);
end

release(viewer)
end
