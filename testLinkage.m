global masterCluster;
global mastersrc;
global vari;
vari = 1;
masterCluster = {1; 1};
img = imread('sobelTest.jpg');
imgEdit = cutMask(img,125,900 -0 ,125,700 -0);
%imgEdit = img>55;
[srcArrayX,srcArrayY] = find(imgEdit >55);
srcArray = cat(2,srcArrayX,srcArrayY);
mastersrc = srcArray;


%{
for i = 1: size(imgEdit,1)
    for j = 1: size(imgEdit,2)
        if imgEdit(i,j) == 1
           srcArry{srcCnt, 1} = i;
           srcArry{srcCnt, 2} = j;
           srcCnt = srcCnt + 1;
        end
    end
end
%}
%distRay = createDistRay(srcArray);
%[distRay,srcArray] = mergeSort(distRay,srcArray);
p = singleLinkageClust(distRay,srcArray);
masterCluster =  masterCluster(~cellfun('isempty',masterCluster))
Mat=masterCluster{6}


plot(Mat(:,1),Mat(:,2),'o')
temp = {};
 for k =  4 :size(masterCluster,1)
    yPt =  min(masterCluster{k}(:,1));
    xPt = min(masterCluster{k}(:,2));
   height = max(masterCluster{k}(:,1)) - yPt;
   width = max(masterCluster{k}(:,2)) - xPt;
    temp{k} = Box(xPt,yPt,width,height);
    [a,s,d,f] = temp{k}.data();
    if width < 0.300 || height < 300
   % imgEdit = insertShape(imgEdit,'rectangle', [a,s,d,f] );
   % imshow((imgEdit>55) * 255 )
    end
 end
%imshow((imgEdit>55) * 255 )
%plot(srcArray(:,1)),srcArray(:,2)
function [y,origSrc] = mergeSort(distX,normArry)
% x is a vector.
% y is a vector consisting of the values in x sorted from
% smallest to largest.
n = length(distX);
if n==1
y = distX;
origSrc = normArry;%nothing changes
else
m = floor(n/2);
% Sort the first half..
[y1,origSrc1] = mergeSort(distX(1:m),normArry(1:m,:)); % values displayed are the values returned by this call of mergeSort
% Sort the second half...
[y2,origSrc2] = mergeSort(distX(m+1:n),normArry(m+1:n,:)); % values displayed are the values returned by this call of mergeSort
% Merge...
[y,origSrc] = merge(y1,y2,origSrc1,origSrc2); % values displayed are the values returned by this call of merge
end
end
function [z,origSrc] = merge(x,y,origSrcX,origSrcY)
% x is a row n-vector with x(1) <= x(2) <= ... <= x(n)
% y is a row m-vector with y(1) <= y(2) <= ... <= y(m)
% z is a row (m+n)-vector comprised of all the values in x and
% y and sorted so that z(1) <= ... <= z(m+n)
n = length(x); m = length(y); z = zeros(1,n+m);
ix = 1; % The index of the next x-value to select.
iy = 1; % The index of the next y-value to select.
for iz=1:(n+m)
% Deteremin the iz-th value for the merged array...
if ix > n
% All done with x-values. Select the next y-value.
z(iz) = y(iy); 
origSrc(iz,:) = origSrcY(iy,:);
iy = iy+1;
elseif iy>m
% All done with y-values. Select the next x-value.
z(iz) = x(ix); 
origSrc(iz,:) = origSrcX(ix,:);
ix = ix + 1;
elseif x(ix) <= y(iy)
% The next x-value is less than or equal to the next y-value
z(iz) = x(ix); 
origSrc(iz,:) = origSrcX(ix,:);
ix = ix + 1;
else
% The next y-value is less than the next x-value
z(iz) = y(iy); 
origSrc(iz,:) = origSrcY(iy,:);
iy = iy + 1;
end
end
end
function distRay = createDistRay(x)
orig = [0;0]
distRay = sqrt(((x(:,1)).^2)+((x(:,2)).^2)); % dist array from origin
end
function out = cutMask(img,x1,y1,x2,y2)
  img(1:x1,:) = 0;
     img(end - x2:end,:) = 0;
      img(:, 1:y1) = 0;
      img(:, end - y2 : end) = 0;
      out = img;
end
function [newClust,clusterGrp] = singleLinkageClust(avgPt)
%d(X,Y) = min d(x,y)
global mastersrc;
if length(avgPt) ==1
    newClust = avgPt;
    clusterGrp = avgPt
elseif length(masterSrc) 
        
else
    %cluster = mastersrc;
    init,prevGrp = singleListClust(avgPt);
    distArray = pdist2(init, mastersrc)
    k = find(distArray == min(distArray),1);
    newClust = clusterMerge(init,mastersrc(k,:),prevGrp);
    
end

%mini= find(pdist2(srcArray(1,:),srcArray(200,:))
end
function [newAvg,joinedCluster] = clusterMerge(pt1,pt2)
    %%{
global masterCluster;
    global vari;
    dist = pdist2(pt1,pt2);
   if dist <= 5
        %add cluster to master cluster
       % joinedCluster = cat(1,clust1,clust2);
      % masterCluster{vari} = cat(1,clust1,clust2);
   else
        vari = vari + 1;
       % joinedCluster = clust2;
   end
   %}
    joinedCluster = cat(1,clust1,clust2);
     newAvg = (avg1 +avg2)/2;
end