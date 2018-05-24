% start program 
  %%{
s = serial('COM7');
fopen(s);
pause(4);
disp('serialready')
fprintf(s,'1');
scan = {};
picArray={};
picArrayG={};
cameraEn = 0;
while 1==1

fscanf(s)
pause(1);
%strcmp(fscanf(s),'trigger')
scan{1,end+1} = strtrim(fscanf(s))
%if isnumeric(scan{1,end})==0
%    scan{2,end} =strcmp(strtrim(scan{end}),'trigger');
if  strcmp(scan{1,end},'trigger') 
    picArray{end+1}=takePic(0,2);
  picArrayG{end+1}=takePic(0,1);
  scan{1,end+1} = 'space';
 %elseif 

end
if strcmp(scan{1,end},'done')
    
    
    
      fclose(s); 
     break;
end
end
%}

   % picArray{1} = picArray{1}(picArray{1}< 10000);
   %picArrayG{2} = takePic(5, 1);
   % picArray{2} = takePic(0,2);
   % picArray{2} = picArray{2}(picArray{2}< 10000);
%img = getDiffPic(picArray{1}, picArray{2}, 10);


img2 = stitch2imgColored(picArrayG);
imshow(img2)
%img2 = rgb2gray(img2);
img5 = sobel(img2);
imshow(img5)
%img = edge(img,'sobel');
%img = edge(img,'canny');
%img2= edge(img2,'sobel');
%img = edge(img2,'canny');
hold on;
%%imshow(cutMask(img2> 55,125,900,125,500));
%plot3D(img * 20);
%img = stitch2imgColored(picArray);

%imshow(img2);
%img = rgb2gray(img);
%imshow(img);


function out = cutMask(img,x1,y1,x2,y2)
  img(1:x1,:) = 0;
     img(end - x2:end,:) = 0;
      img(:, 1:y1) = 0;
      img(:, end - y2 : end) = 0;
      out = img;
end
function plotted = plot3D(img2)

imshow(img2*20)
surf(1:size(img2,2),1:size(img2,1),img2)
end
function scanned = takePic(timeDelay,cameraType)
        depthVid = videoinput('kinect', cameraType);%starts video camera
        depthSource = getselectedsource(depthVid);
        framesPerTrig = 1;%on every start() one frame/picture is captured by the kinect
        depthVid.FramesPerTrigger = framesPerTrig;%assigns ^ variable to function in object
      %  preview(depthVid);%starts preview for user feed back
      disp('taking picture')
        pause(timeDelay);

       start(depthVid);
      [depthFrameData,depthTimeData,depthMetaData] = getdata(depthVid);
 
     
      scanned = depthFrameData(:,:,:,framesPerTrig);
      %{
    
 %}
      %scanned(scanned > 1000 | scanned < 700) = 0;
      
      
      stop(depthVid);
     % stoppreview(depthVid);
     disp('picture taken')
       %outputs a picture
end
function mask = getDiffPic(log2,log1,thrsh)
%log 1 is without the dent
%log 2 is with the dent
%thrsh is a threshold for overlapping the 
 
    compare = ~(log1 +thrsh > log2 & log1-thrsh<log2)
    for i=1:size(log1,1)%424 pixels wide
        for j=1:size(log1,2)%512 pixels long
            if compare(i,j)==1
                mask(i,j)=log2(i,j);
            else 
                mask(i,j)=0;
            end
        end
    end
    %masks out only the 
end
function stch = stitch2img(imgArray)

%% Step 2 - Register Image Pairs 
% To create the panorama, start by registering successive image pairs using
% the following procedure:
%
% # Detect and match features between $I(n)$ and $I(n-1)$.
% # Estimate the geometric transformation, $T(n)$, that maps $I(n)$ to $I(n-1)$.
% # Compute the transformation that maps $I(n)$ into the panorama image as $T(n) * T(n-1) * ... * T(1)$.

% Read the first image from the image set.
I = imgArray{1};

% Initialize features for I(1)
grayImage = I;
points = detectSURFFeatures(grayImage);
[features, points] = extractFeatures(grayImage, points);

% Initialize all the transforms to the identity matrix. Note that the
% projective transform is used here because the building images are fairly
% close to the camera. Had the scene been captured from a further distance,
% an affine transform would suffice.
numImages = size(imgArray,2);
tforms(numImages) = projective2d(eye(3));

% Iterate over remaining image pairs
for n = 2:numImages
    
    % Store points and features for I(n-1).
    pointsPrevious = points;
    featuresPrevious = features;
        
    % Read I(n).
    I = imgArray{n};
    
    % Detect and extract SURF features for I(n).
    grayImage = I;    
    points = detectSURFFeatures(grayImage);    
    [features, points] = extractFeatures(grayImage, points);
  
    % Find correspondences between I(n) and I(n-1).
    indexPairs = matchFeatures(features, featuresPrevious, 'Unique', true);
       
    matchedPoints = points(indexPairs(:,1), :);
    matchedPointsPrev = pointsPrevious(indexPairs(:,2), :);        
    
    % Estimate the transformation between I(n) and I(n-1).
    tforms(n) = estimateGeometricTransform(matchedPoints, matchedPointsPrev,...
        'projective', 'Confidence', 99.9, 'MaxNumTrials', 5000);
    
    % Compute T(n) * T(n-1) * ... * T(1)
    tforms(n).T = tforms(n).T * tforms(n-1).T; 
end

%%
% At this point, all the transformations in |tforms| are relative to the
% first image. This was a convenient way to code the image registration
% procedure because it allowed sequential processing of all the images.
% However, using the first image as the start of the panorama does not
% produce the most aesthetically pleasing panorama because it tends to
% distort most of the images that form the panorama. A nicer panorama can
% be created by modifying the transformations such that the center of the
% scene is the least distorted. This is accomplished by inverting the
% transform for the center image and applying that transform to all the
% others.
%
% Start by using the |projective2d| |outputLimits| method to find the
% output limits for each transform. The output limits are then used to
% automatically find the image that is roughly in the center of the scene.

imageSize = size(I);  % all the images are the same size

% Compute the output limits  for each transform
for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(2)], [1 imageSize(1)]);    
end

%%
% Next, compute the average X limits for each transforms and find the image
% that is in the center. Only the X limits are used here because the scene
% is known to be horizontal. If another set of images are used, both the X
% and Y limits may need to be used to find the center image.

avgXLim = mean(xlim, 2);

[~, idx] = sort(avgXLim);

centerIdx = floor((numel(tforms)+1)/2);

centerImageIdx = idx(centerIdx);

%%
% Finally, apply the center image's inverse transform to all the others.

Tinv = invert(tforms(centerImageIdx));

for i = 1:numel(tforms)    
    tforms(i).T = tforms(i).T * Tinv.T;
end

%% Step 3 - Initialize the Panorama
% Now, create an initial, empty, panorama into which all the images are
% mapped. 
% 
% Use the |outputLimits| method to compute the minimum and maximum output
% limits over all transformations. These values are used to automatically
% compute the size of the panorama.

for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(2)], [1 imageSize(1)]);
end

% Find the minimum and maximum output limits 
xMin = min([1; xlim(:)]);
xMax = max([imageSize(2); xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([imageSize(1); ylim(:)]);

% Width and height of panorama.
width  = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the "empty" panorama.
stch = zeros([height width 1], 'like', I);

%% Step 4 - Create the Panorama
% Use |imwarp| to map images into the panorama and use
% |vision.AlphaBlender| to overlay the images together.

blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');  

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);

% Create the panorama.
for i = 1:numImages
    
    I = imgArray{i};   
   
    % Transform I into the panorama.
    warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
                  
    % Generate a binary mask.    
    mask = imwarp(true(size(I,1),size(I,2)), tforms(i), 'OutputView', panoramaView);
    
    % Overlay the warpedImage onto the panorama.
    stch = step(blender, stch, warpedImage, mask);
end
%stch = stch;%The Image 1 and Image 2 inputs must have the same number of dimensions.




end  
function stch = stitch2imgColored(imgArray)

%% Step 2 - Register Image Pairs 
% To create the panorama, start by registering successive image pairs using
% the following procedure:
%
% # Detect and match features between $I(n)$ and $I(n-1)$.
% # Estimate the geometric transformation, $T(n)$, that maps $I(n)$ to $I(n-1)$.
% # Compute the transformation that maps $I(n)$ into the panorama image as $T(n) * T(n-1) * ... * T(1)$.

% Read the first image from the image set.
I = imgArray{1};

% Initialize features for I(1)

grayImage = rgb2gray(I);
points = detectSURFFeatures(grayImage);
[features, points] = extractFeatures(grayImage, points);

% Initialize all the transforms to the identity matrix. Note that the
% projective transform is used here because the building images are fairly
% close to the camera. Had the scene been captured from a further distance,
% an affine transform would suffice.
numImages = size(imgArray,2);
tforms(numImages) = projective2d(eye(3));

% Iterate over remaining image pairs
for n = 2:numImages
    
    % Store points and features for I(n-1).
    pointsPrevious = points;
    featuresPrevious = features;
        
    % Read I(n).
    I = imgArray{n};
    
    % Detect and extract SURF features for I(n).
   grayImage = rgb2gray(I);    
    points = detectSURFFeatures(grayImage);    
    [features, points] = extractFeatures(grayImage, points);
  
    % Find correspondences between I(n) and I(n-1).
    indexPairs = matchFeatures(features, featuresPrevious, 'Unique', true);
       
    matchedPoints = points(indexPairs(:,1), :);
    matchedPointsPrev = pointsPrevious(indexPairs(:,2), :);        
    
    % Estimate the transformation between I(n) and I(n-1).
    tforms(n) = estimateGeometricTransform(matchedPoints, matchedPointsPrev,...
        'projective', 'Confidence', 99.9, 'MaxNumTrials', 5000);
    
    % Compute T(n) * T(n-1) * ... * T(1)
    tforms(n).T = tforms(n).T * tforms(n-1).T; 
end

%%
% At this point, all the transformations in |tforms| are relative to the
% first image. This was a convenient way to code the image registration
% procedure because it allowed sequential processing of all the images.
% However, using the first image as the start of the panorama does not
% produce the most aesthetically pleasing panorama because it tends to
% distort most of the images that form the panorama. A nicer panorama can
% be created by modifying the transformations such that the center of the
% scene is the least distorted. This is accomplished by inverting the
% transform for the center image and applying that transform to all the
% others.
%
% Start by using the |projective2d| |outputLimits| method to find the
% output limits for each transform. The output limits are then used to
% automatically find the image that is roughly in the center of the scene.

imageSize = size(I);  % all the images are the same size

% Compute the output limits  for each transform
for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(2)], [1 imageSize(1)]);    
end

%%
% Next, compute the average X limits for each transforms and find the image
% that is in the center. Only the X limits are used here because the scene
% is known to be horizontal. If another set of images are used, both the X
% and Y limits may need to be used to find the center image.

avgXLim = mean(xlim, 2);

[~, idx] = sort(avgXLim);

centerIdx = floor((numel(tforms)+1)/2);

centerImageIdx = idx(centerIdx);

%%
% Finally, apply the center image's inverse transform to all the others.

Tinv = invert(tforms(centerImageIdx));

for i = 1:numel(tforms)    
    tforms(i).T = tforms(i).T * Tinv.T;
end

%% Step 3 - Initialize the Panorama
% Now, create an initial, empty, panorama into which all the images are
% mapped. 
% 
% Use the |outputLimits| method to compute the minimum and maximum output
% limits over all transformations. These values are used to automatically
% compute the size of the panorama.

for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(2)], [1 imageSize(1)]);
end

% Find the minimum and maximum output limits 
xMin = min([1; xlim(:)]);
xMax = max([imageSize(2); xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([imageSize(1); ylim(:)]);

% Width and height of panorama.
width  = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the "empty" panorama.
stch = zeros([height width 3], 'like', I);

%% Step 4 - Create the Panorama
% Use |imwarp| to map images into the panorama and use
% |vision.AlphaBlender| to overlay the images together.

blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');  

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width ], xLimits, yLimits);

% Create the panorama.
for i = 1:numImages
    
    I = imgArray{i};   
   
    % Transform I into the panorama.
    warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
                  
    % Generate a binary mask.    
    mask = imwarp(true(size(I,1),size(I,2)), tforms(i), 'OutputView', panoramaView);
    
    % Overlay the warpedImage onto the panorama.
    stch = step(blender, stch, warpedImage, mask);
end
%stch = stch;%The Image 1 and Image 2 inputs must have the same number of dimensions.




end  
function [ lenaOutput] = sobel(X)

%X = imread('jordanpic.jpg');

X= double(X); height = size(X, 1); width = size(X, 2); channel = size(X, 3);

lenaOutput = X;

Gx = [1 +2 +1; 0 0 0; -1 -2 -1]; Gy = Gx';

for i = 2 : height-1

   for j = 2 : width-1  
       for k = 1 : channel
           tempLena = X(i - 1 : i + 1, j - 1 : j + 1, k);
           a=(sum(Gx.* tempLena));
           x = sum(a);
           b= (sum(Gy.* tempLena));
            y = sum(b);
           pixValue =sqrt(x.^2+ y.^2);
          % pixValue =(x-y);
           lenaOutput(i, j, k) = pixValue;
       end 
   end
end

lenaOutput = uint8(lenaOutput); figure; imshow(abs(lenaOutput),[]); title(' Sobel Edge Detection');
end