
%% cubeDim is 16  88%   the result is very good!!!



poolDim = 4;
patchDim1 = 6;
patchDim2 = 10;
cubeDim = 25;

hiddenSize = 40;     % number of hidden units 
patchNumber1 = 40000;
patchNumber2 = 40000;

visibleSize1 = power(patchDim1,3);   % number of input units 
visibleSize2 = power(patchDim2,3);   % number of input units 


convolvedDim1 = cubeDim - patchDim1 +1;
convolvedDim2 = cubeDim - patchDim2 +1;

threshold = 1;

inputDataDim = 9; %7 6


%chairNumber = [2, 5, 7, 11, 14, 17, 18, 20, 21, 22, 24, 25, 29, 30, 31, 33, 36, 37, 38, 40, 42, 51, 54, 57, 62, 65, 68, 79, 86, 128, 142,];
chairNumber = [1, 4, 5, 8, 9, 10, 12, 14, 15, 16, 18, 19, 21, 23, 24, 28, 29, 31, 32, 34, 35, 38, 50, 53, 54, 60, 61, 65, 80, 87, 130, 136, 137, 139, 140];
label = [1 1 0 1 1 1 0 1 1 0 1 1 1 1 1 1 1 1 1 1 1 0 0 1 1 1 0 0 0 0 0];
[m,n] = size(chairNumber);
predClass = zeros(1,n);
result = 2*ones(1,n);


stepNum = 1;
stepSize = hiddenSize;
dataSize = 1;

load /scratch/uceedua/DL3D/four_angle_right/cnn3D_2filters_25_4/optTheta_patch1.mat;
load /scratch/uceedua/DL3D/four_angle_right/cnn3D_2filters_25_4/optTheta_patch2.mat;

load /scratch/uceedua/DL3D/four_angle_right/cnn3D_2filters_25_4/softmaxModel.mat;

for i = 1:n
    %file_in = fopen('/scratch/uceedua/data/chair/chair_a3.txt','r');
    file_in = ['/scratch/uceedua/object_segmentation/object/object_' num2str(chairNumber(i)) '.txt'];
    file_out = '/scratch/uceedua/data/chair/chair_a.mat';
    testData = voxel_normalise_pointcloud(file_in,cubeDim,file_out,threshold,inputDataDim);

    pooledFeaturesTest = zeros(hiddenSize,1,power(convolvedDim1/ poolDim,3)+power(convolvedDim2/ poolDim,3));

    pooledFeaturesTest1 = conv_pool_pointcloud(stepNum,stepSize,poolDim,patchDim1,cubeDim,hiddenSize,opttheta1,testData(:));
    pooledFeaturesTest2 = conv_pool_pointcloud(stepNum,stepSize,poolDim,patchDim2,cubeDim,hiddenSize,opttheta2,testData(:));
    pooledFeaturesTest(:,:,1:power(convolvedDim1/ poolDim,3)) = reshape(pooledFeaturesTest1,[stepSize, dataSize,power(convolvedDim1/ poolDim,3)]);
    pooledFeaturesTest(:,:,power(convolvedDim1/ poolDim,3)+1:end) = reshape(pooledFeaturesTest2,[stepSize, dataSize,power(convolvedDim2/ poolDim,3)]);


    softmaxX = permute(pooledFeaturesTest, [1 3 2]);
    softmaxX = reshape(softmaxX, [numel(pooledFeaturesTest) / dataSize, dataSize]);

    [predClass(i),predValue(i)] = softmaxPredict(softmaxModel, softmaxX);
    
    if predClass(i) == 3
        result(i) = 1;
    end
    
end


% rightNum = 0;
% chairNum = 0;
% for i = 1:size(label,2)
%     if (result(i) == label(i))
%        rightNum = rightNum +1;
%     end
%     if label(i) == 1
%         chairNum = chairNum + 1;
%     end
% end
% 
% rightNum/chairNum


predClass
predValue
%label







