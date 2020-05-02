function [pooledFeaturesTrain, pooledFeaturesTest] = conv_pool_4(stepNum,stepSize,poolDim,patchDim,cubeDim,hiddenSize,opttheta)

[trainData,trainLabels] = pretraining_trainData_4(cubeDim);
[testData,testLabels] = pretraining_testData_4(cubeDim);


convolvedDim = cubeDim - patchDim +1;


visibleSize = power(patchDim,3);   % number of input units 



W = reshape(opttheta(1:visibleSize * hiddenSize), [hiddenSize, visibleSize]);
b = opttheta(2*hiddenSize*visibleSize+1:2*hiddenSize*visibleSize+hiddenSize);


%% Convolve and pool


startEnd = [stepSize*(stepNum-1)+1,stepSize*stepNum];



 pooledFeaturesTrain = zeros(stepSize, size(trainData,2),convolvedDim/ poolDim, convolvedDim/ poolDim,convolvedDim/ poolDim);
 pooledFeaturesTest = zeros(stepSize, size(testData,2),convolvedDim/ poolDim, convolvedDim/ poolDim,convolvedDim/ poolDim);

 

    
    
    featureStart = startEnd(1);
    featureEnd = startEnd(2);
    
    fprintf('Step: features %d to %d\n', featureStart, featureEnd);  
    Wt = W(featureStart:featureEnd, :);
    bt = b(featureStart:featureEnd);    
    

    
    fprintf('Convolving and pooling train images\n');
    convolvedFeaturesThis = cnnConvolve3D(patchDim, stepSize, cubeDim, trainData, Wt, bt);
    pooledFeaturesThis = cnnPool3D(poolDim, convolvedFeaturesThis);
    pooledFeaturesTrain(featureStart:featureEnd, :, :, :, :) = pooledFeaturesThis;   
    
    
    clear convolvedFeaturesThis pooledFeaturesThis;
    
    fprintf('Convolving and pooling test images\n');
    convolvedFeaturesThis = cnnConvolve3D(patchDim, stepSize, cubeDim, testData, Wt, bt);
    pooledFeaturesThis = cnnPool3D(poolDim, convolvedFeaturesThis);
    pooledFeaturesTest(featureStart:featureEnd, :, :, :, :) = pooledFeaturesThis;   
   


    clear convolvedFeaturesThis pooledFeaturesThis;







end
