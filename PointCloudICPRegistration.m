% Register Personal Kinect Data with Built In ICP Implementations
% 10/22/2015

function [] = mainDepthRegistration()
    %% Initialize Video Reader
    dataFilePath = './InputData';
    rgbVidReader = VideoReader([dataFilePath, '/rgbData.avi']);
    numFrames = get(rgbVidReader, 'NumberOfFrames'); % Get the Number of Frames.
    fidDepth = fopen([dataFilePath, '/depthData.bin'], 'r');
    if ~fidDepth
        disp('Failed to Open .bin File');
        return;
    end
     %% Initialize Video Writer
    slamVideoPath = './';
    videoName = 'ICPReconstruction';
    slamVideoWriter = VideoWriter([slamVideoPath, '/', videoName, '.avi']);
    slamVideoWriter.FrameRate = 30;
    open(slamVideoWriter);
    %% Initialize 'Last Frame' as the First Frame
    gridSize = 100; % mm
    mergeSize = 15; % mm
    % Get the Next RGB Frame
    lastRGBFrame = (read(rgbVidReader, 2)); % Read the Second Frame
    lastRGBFrame = switchRGB(lastRGBFrame);
    lastRGBMap = generateMx3Color(lastRGBFrame);
    lastDepthFrame = (fread(fidDepth, [640 480], 'uint16'))'; 
    lastDepthFrame = (fread(fidDepth, [640 480], 'uint16'))'; % Read the Second Frame
    focalLength = computeFocalLength(lastDepthFrame); % In Pixels :]
    [lastVertexMap, lastVertexMap2D] = convertDepth2VertextMap(lastDepthFrame, focalLength);
    lastFramePointCloud = pointCloud(lastVertexMap', 'Color', uint8(lastRGBMap)); % Convert to a PointCoud Class
    lastFramePointCloud = pcdownsample(lastFramePointCloud, 'gridAverage', gridSize);
    ptCloudScene = lastFramePointCloud;
    % Initialize Position Array
    globalOrigin = initGlobalOrigin();
    %% Visualize First Frame (at Global Origin Position)
    viewAz = -135;
    viewEl = 35;
    figure('Position',get(0,'ScreenSize'));
    colormap(hot);
    subplot(2, 2, 2);
    imshow(lastRGBFrame);
    title('RGB Image from Kinect', 'FontWeight', 'bold', 'FontSize', 12);
    axis equal tight off;
    % Depth Image
    subplot(2, 2, 4);
    imagesc(lastDepthFrame, [0 7000]);
    title('Calibrated/Registered Depth Image from Kinect (mm)', 'FontWeight', 'bold', 'FontSize', 12);
    axis equal tight off;
    colorbar;
    axRecon = subplot(2, 2, [1, 3]);
     % Display the Origin of the Coordinate System
    quiver3( [0,0,0],   [0,0,0],   [0,0,0], ...
            [250,0,0], [0,250,0], [0,0,250], 'color', [0 0 0], 'LineWidth', 2);
    hold on;
    pcshow(ptCloudScene);
    title('Texture Mapped Vertices Projected Registered wrt. Global Origin', 'FontWeight', 'bold', 'FontSize', 12);
    set(axRecon, 'Zdir', 'reverse'); % Reverse Z Direction
    set(axRecon, 'Ydir', 'reverse'); % Reverse Y Direction
    xlabel('X-Axis (mm)', 'FontWeight', 'bold');
    ylabel('Y-Axis (mm)', 'FontWeight', 'bold');
    zlabel('Z-Axis (mm)', 'FontWeight', 'bold');
    axis equal;
    grid on;
    hold off;
    view(viewAz, viewEl);
    drawnow;
    frame = getframe(gcf);
    % writeVideo(slamVideoWriter,frame)
    for iFrame = 3:3:numFrames
        %% Initialize the Point Cloud of the Current Time-Step 
        % Get the Next RGB Frame
        thisRGBFrame = (read(rgbVidReader, iFrame));
        thisRGBFrame = switchRGB(thisRGBFrame);
        thisRGBMap = generateMx3Color(thisRGBFrame);
        % Get Matrices for 3D Depth Measurement
        thisDepthFrame = (fread(fidDepth, [640 480], 'uint16'))'; % Read the Second Frame
        thisDepthFrame = (fread(fidDepth, [640 480], 'uint16'))'; % Read the Second Frame
        thisDepthFrame = (fread(fidDepth, [640 480], 'uint16'))'; % Read the Second Frame
        [thisVertexMap] = convertDepth2VertextMap(thisDepthFrame, focalLength);
        thisFramePointCloud = pointCloud(thisVertexMap', 'Color', uint8(thisRGBMap)); % Convert to a PointCoud Class
        thisFramePointCloud = pcdownsample(thisFramePointCloud, 'gridAverage', gridSize);
        %% Find Rigid Body Transformation between Current Point Cloud and the Last Point Cloud
        %                        Moving                Reference
        tform = pcregrigid(thisFramePointCloud, lastFramePointCloud, 'Metric','pointToPlane','Extrapolate', true);
        if iFrame == 3
            accumTform = tform;
        else
            accumTform = affine3d(tform.T * accumTform.T);
        end
        thisFramePointCloudAligned = pctransform(thisFramePointCloud, accumTform);
        ptCloudScene = pcmerge(ptCloudScene, thisFramePointCloudAligned, mergeSize);
        lastFramePointCloud = thisFramePointCloud;
        %% Visualize the Updated Point Cloud Scene
        updateVisualization(globalOrigin, accumTform, ptCloudScene, thisRGBFrame, thisDepthFrame, iFrame);
        drawnow;
        frame = getframe(gcf);
        writeVideo(slamVideoWriter,frame)
        disp(iFrame);
    end
end

%% Initialize Global Origin Coordinate System
function [gloabalOrigin] = initGlobalOrigin()
    % Position
    gloabalOrigin.Pos = [0; 
                         0; 
                         0; 
                         1];
    %                            X  Y  Z
    gloabalOrigin.viewingAxes = [250,  0,  0; 
                                 0,   250, 0; 
                                 0,    0, 250; 
                                 0,    0,  0];
    return;
end

%% Switch RGB Components
function [thisRGBFrame] = switchRGB(thisRGBFrame)
    tmpMtx = thisRGBFrame;
    thisRGBFrame(:,:,1) = tmpMtx(:,:,3);
    thisRGBFrame(:,:,3) = tmpMtx(:,:,1);
end

%% Compute Focal Length
function [focalLength] = computeFocalLength(depthFrame)
    %% Kinect Internal Camera Parameters
    fov.hor = 57;
    fov.vert = 43;
%     fov.hor = 62;
%     fov.vert = 48.6;
    deg2rad = pi/180;
    imgLength = size(depthFrame, 2);
    imgHeight = size(depthFrame, 1);
    focalLength.x = imgLength/2/(tan(deg2rad*fov.hor/2)); % In pixels
    focalLength.y = imgHeight/2/(tan(deg2rad*fov.vert/2)); % In pixels
    return;
end


%% Generate Color Point Cloud Vector as M by 3 Matrix
function [thisRGBMap] = generateMx3Color(thisRGBFrame)
    numRows = size(thisRGBFrame, 1);
    numCols = size(thisRGBFrame, 2);
    thisRGBMap = zeros(numRows*numCols, 3);
    tmpMtx = thisRGBFrame(:,:,1);
    thisRGBMap(:,1) = tmpMtx(:);
    tmpMtx = thisRGBFrame(:,:,2);
    thisRGBMap(:,2) = tmpMtx(:);
    tmpMtx = thisRGBFrame(:,:,3);
    thisRGBMap(:,3) = tmpMtx(:);
end


%% Project Depth Maps to Vertex Maps (as a [3xN] Matrix)
function [vertexMapMtx, vertexMap] = convertDepth2VertextMap(depthFrame, focalLength)
    %% Map Depth Image Values to 3D Points
    imgLength = size(depthFrame, 2);
    imgHeight = size(depthFrame, 1);
    vertexMap = zeros(imgHeight, imgLength, 3); % [X Y Z]
%     centeredRow = [imgHeight:-1:1]-ceil(imgHeight/2);
    centeredRow =  [-imgHeight:1:-1]+ceil(imgHeight/2);
    centeredCol =  [1:imgLength]-ceil(imgLength/2);
    [centeredColMesh, centeredRowMesh] = meshgrid(centeredCol, centeredRow);
    %% Fill in Vertex Map
    % Turn 0s to NaN for Better 3D Viewing
    depthFrame((depthFrame < 200)) = NaN;
    % 1. X Axis: Forward Distance along Imaging Axis
    vertexMap(:,:,1) = depthFrame;
    % 2. Y Axis: Lateral/Right Distance
    vertexMap(:,:,2) = depthFrame.*centeredColMesh./focalLength.x; 
    % 2. Z Axis: Downward/Down Distance
    vertexMap(:,:,3) = depthFrame.*centeredRowMesh./focalLength.y; 
    %% Set Up both Depth Images as [3xN] Coordinates in (their respective) Camera Space
    vertexMapMtx = zeros(3, imgLength*imgHeight);
    tmpRowMtx = vertexMap(:,:, 1);
    vertexMapMtx(1,:) = tmpRowMtx(:);
    tmpRowMtx = vertexMap(:,:, 2);
    vertexMapMtx(2,:) = tmpRowMtx(:);
    tmpRowMtx = vertexMap(:,:, 3);
    vertexMapMtx(3,:) = tmpRowMtx(:);
    return;
end

%% Update Visualization
function [] = updateVisualization(globalOrigin, accumTform, ptCloudScene, thisRGBFrame, thisDepthFrame, iFrame)
    % RGB Image
    subplot(2, 2, 2);
    imshow(thisRGBFrame);
    title('RGB Image from Kinect', 'FontWeight', 'bold', 'FontSize', 12);
    axis equal tight off;
    % Depth Image
    subplot(2, 2, 4);
    imagesc(thisDepthFrame, [0 7000]);
    title(['Calibrated/Registered Depth Image from Kinect (mm) at Frame: ', num2str(iFrame)], 'FontWeight', 'bold', 'FontSize', 12);
    axis equal tight off;
    colorbar;
    %% Plot the Updated Reconstruction 
    axRecon = subplot(2, 2, [1 3]);
    % Display the Origin of the Coordinate System
    quiver3([0,0,0], [0,0,0], [0,0,0], ...
            [250,0,0], [0,250,0], [0,0,250], 'color', [0 0 0], 'LineWidth', 2);
     hold on;
     currPos = accumTform.T'*globalOrigin.Pos;
     currViewingAxes = accumTform.T'*globalOrigin.viewingAxes;
     pcshow(ptCloudScene);
     % Display the Current Coordiante System wrt. the Global Coordinate System
        quiver3(  [currPos(1)], [currPos(2)],  [currPos(3)], ...    
                   currViewingAxes(1,1), currViewingAxes(2,1), currViewingAxes(3,1), 'color', [1 0 0], 'LineWidth', 2);         
        quiver3(  [currPos(1)], [currPos(2)],  [currPos(3)], ...    
                   currViewingAxes(1,2), currViewingAxes(2,2), currViewingAxes(3,2), 'color', [0 1 0], 'LineWidth', 2); 
        quiver3(  [currPos(1)], [currPos(2)],  [currPos(3)], ...    
                   currViewingAxes(1,3), currViewingAxes(2,3), currViewingAxes(3,3), 'color', [0 0 1], 'LineWidth', 2);    
     title('Texture Mapped Vertices Projected wrt. Current Position', 'FontWeight', 'bold', 'FontSize', 12);
     set(axRecon, 'Ydir', 'reverse'); % Reverse Y Direction
     set(axRecon, 'Zdir', 'reverse'); % Reverse Z Direction
     xlabel('X-Axis (mm)', 'FontWeight', 'bold');
     ylabel('Y-Axis (mm)', 'FontWeight', 'bold');
     zlabel('Z-Axis (mm)', 'FontWeight', 'bold');
     axis equal;
     grid on;
     hold off;
     view(-135, 35);
     disp(ptCloudScene.Count);
     return;
end



