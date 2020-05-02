





%% parameters


poolDim = 4;
patchDim1 = 6;
patchDim2 = 10;
cubeDim = 25;

hiddenSize = 40;     % number of hidden units 
patchNumber1 = 40000;
patchNumber2 = 40000;



sparsityParam = 0.01;   % desired average activation of the hidden units.
                     % (This was denoted by the Greek alphabet rho, which looks like a lower-case "p",
		     %  in the lecture notes). 
lambda = 0.0001;     % weight decay parameter       
beta = 3;            % weight of sparsity penalty term   


convolvedDim1 = cubeDim - patchDim1 +1;
convolvedDim2 = cubeDim - patchDim2 +1;

visibleSize1 = power(patchDim1,3);   % number of input units 
visibleSize2 = power(patchDim2,3);   % number of input units 
%% Data

%patches = pretraining_patch(cubeDim,patchDim,patchNumber);
load optTheta_patch1.mat;  %obtained from autoencoder_patch_4.m
load optTheta_patch2.mat;

[trainData,trainLabels] = pretraining_trainData_4(cubeDim);
[testData,testLabels] = pretraining_testData_4(cubeDim);


W1 = reshape(opttheta1(1:visibleSize1 * hiddenSize), [hiddenSize, visibleSize1]);
b1 = opttheta1(2*hiddenSize*visibleSize1+1:2*hiddenSize*visibleSize1+hiddenSize);

W2 = reshape(opttheta2(1:visibleSize2 * hiddenSize), [hiddenSize, visibleSize2]);
b2 = opttheta2(2*hiddenSize*visibleSize2+1:2*hiddenSize*visibleSize2+hiddenSize);

%% Convolve and pool
% %test
% convCubes = trainData(:,1:10);
% convolvedFeatures = cnnConvolve3D(patchDim, hiddenSize, cubeDim, convCubes, W, b);
% pooledFeatures = cnnPool3D(poolDim, convolvedFeatures);


stepSize = 1;


 pooledFeaturesTrain1 = zeros(hiddenSize, size(trainData,2),convolvedDim1/ poolDim, convolvedDim1/ poolDim,convolvedDim1/ poolDim);
 pooledFeaturesTest1 = zeros(hiddenSize, size(testData,2),convolvedDim1/ poolDim, convolvedDim1/ poolDim,convolvedDim1/ poolDim);

 pooledFeaturesTrain2 = zeros(hiddenSize, size(trainData,2),convolvedDim2/ poolDim, convolvedDim2/ poolDim,convolvedDim2/ poolDim);
 pooledFeaturesTest2 = zeros(hiddenSize, size(testData,2),convolvedDim2/ poolDim, convolvedDim2/ poolDim,convolvedDim2/ poolDim);
 
 
for convPart = 1:(hiddenSize / stepSize)
    
    
    featureStart = (convPart - 1) * stepSize + 1;
    featureEnd = convPart * stepSize;
    
    fprintf('Step %d: features %d to %d\n', convPart, featureStart, featureEnd);  
    Wt1 = W1(featureStart:featureEnd, :);
    bt1 = b1(featureStart:featureEnd);    
    
    Wt2 = W2(featureStart:featureEnd, :);
    bt2 = b2(featureStart:featureEnd);  
    
    fprintf('Convolving and pooling train images\n');
    convolvedFeaturesThis1 = cnnConvolve3D(patchDim1, stepSize, cubeDim, trainData, Wt1, bt1);
    pooledFeaturesThis1 = cnnPool3D(poolDim, convolvedFeaturesThis1);
    pooledFeaturesTrain1(featureStart:featureEnd, :, :, :, :) = pooledFeaturesThis1;   
    
    convolvedFeaturesThis2 = cnnConvolve3D(patchDim2, stepSize, cubeDim, trainData, Wt2, bt2);
    pooledFeaturesThis2 = cnnPool3D(poolDim, convolvedFeaturesThis2);
    pooledFeaturesTrain2(featureStart:featureEnd, :, :, :, :) = pooledFeaturesThis2;
    
    clear convolvedFeaturesThis pooledFeaturesThis;
    
    fprintf('Convolving and pooling test images\n');
    convolvedFeaturesThis1 = cnnConvolve3D(patchDim1, stepSize, cubeDim, testData, Wt1, bt1);
    pooledFeaturesThis1 = cnnPool3D(poolDim, convolvedFeaturesThis1);
    pooledFeaturesTest1(featureStart:featureEnd, :, :, :, :) = pooledFeaturesThis1;   
   
    convolvedFeaturesThis2 = cnnConvolve3D(patchDim2, stepSize, cubeDim, testData, Wt2, bt2);
    pooledFeaturesThis2 = cnnPool3D(poolDim, convolvedFeaturesThis2);
    pooledFeaturesTest2(featureStart:featureEnd, :, :, :, :) = pooledFeaturesThis2; 

    clear convolvedFeaturesThis pooledFeaturesThis;

end
% 
% load pooledFeaturesTrain1
% load pooledFeaturesTrain2
% load pooledFeaturesTest1
% load pooledFeaturesTest2

pooledFeaturesTrain = zeros(hiddenSize,size(trainData,2),power(convolvedDim1/ poolDim,3)+power(convolvedDim2/ poolDim,3));
pooledFeaturesTest = zeros(hiddenSize,size(testData,2),power(convolvedDim1/ poolDim,3)+power(convolvedDim2/ poolDim,3));


pooledFeaturesTrain(:,:,1:power(convolvedDim1/ poolDim,3)) = reshape(pooledFeaturesTrain1,[hiddenSize, size(trainData,2),power(convolvedDim1/ poolDim,3)]);
pooledFeaturesTrain(:,:,power(convolvedDim1/ poolDim,3)+1:end) = reshape(pooledFeaturesTrain2,[hiddenSize, size(trainData,2),power(convolvedDim2/ poolDim,3)]);

pooledFeaturesTest(:,:,1:power(convolvedDim1/ poolDim,3)) = reshape(pooledFeaturesTest1,[hiddenSize, size(testData,2),power(convolvedDim1/ poolDim,3)]);
pooledFeaturesTest(:,:,power(convolvedDim1/ poolDim,3)+1:end) = reshape(pooledFeaturesTest2,[hiddenSize, size(testData,2),power(convolvedDim2/ poolDim,3)]);




save('cnnPooledFeatures.mat', 'pooledFeaturesTrain', 'pooledFeaturesTest');

load cnnPooledFeatures

%% SoftMax

softmaxLambda = 1e-4;
numClasses = 10;

softmaxX = permute(pooledFeaturesTrain, [1 3 2]);
softmaxX = reshape(softmaxX, [numel(pooledFeaturesTrain) / size(trainData,2), size(trainData,2)]);
softmaxY = trainLabels;

options = struct;
options.maxIter = 200;
softmaxModel = softmaxTrain(numel(pooledFeaturesTrain) / size(trainData,2),...
    numClasses, softmaxLambda, softmaxX, softmaxY, options);
save('softmaxModel.mat','softmaxModel');

%% test

softmaxX = permute(pooledFeaturesTest, [1 3 2]);
softmaxX = reshape(softmaxX, [numel(pooledFeaturesTest) / size(testData,2), size(testData,2)]);
softmaxY = testLabels;

[pred] = softmaxPredict(softmaxModel, softmaxX);
acc = (pred(:) == softmaxY(:));
acc = sum(acc) / size(acc, 1);
fprintf('Accuracy: %2.3f%%\n', acc * 100);


