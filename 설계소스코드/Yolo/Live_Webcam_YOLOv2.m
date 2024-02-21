function Live_Webcam_YOLOv21()
% Connect to webcam
wcam = webcam(2);
wcam.Resolution = wcam.AvailableResolutions{1};
viewer = vision.DeployableVideoPlayer();

% Auxiliary variables
fps = 0;
avgfps = [];
cont = true;
% Load your Trained YOLOv2 detector
load yoloTargetCar.mat

while cont
    frame = snapshot(wcam);
    tic; % Count FPS
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
    
    if num > 0
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
         % add anotations to show the result
         disp(bbox1);
        detectedImg = insertObjectAnnotation(detectedImg, 'rectangle', bbox1, annotation,'Color',color);   
    end
    % show the result
    detectedImg = insertText(detectedImg, [1, 1],  sprintf('FPS %2.2f', fps), 'FontSize', 26, 'BoxColor', 'y');    
   
    viewer(detectedImg)
    cont = isOpen(viewer);
end

release(viewer)
end
