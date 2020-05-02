%% 3 auto-encoders and 1 softmax fine-tune for 3D
%% for 4 input viewing angles
%clear all;
%%======================================================================
%% STEP
cubeDim = 16;
inputSize = cubeDim * cubeDim * cubeDim;
numClasses = 10;%need to normalise other objects
hiddenSizeL1 = 500;%can be any number
hiddenSizeL2 = 600;
hiddenSizeL3 = 700;
sparsityParam = 0.1;
lambda = 3e-3;
beta = 3;  

[trainData,trainLabels] = pretraining_trainData_4(cubeDim);
%%======================================================================
%% STEP: Train the first sparse autoencoder

sae1Theta = initializeParameters(hiddenSizeL1, inputSize);

options.Method = 'lbfgs'; 
options.maxIter = 400;	 
options.display = 'on';


[sae1OptTheta, cost] = minFunc( @(p) sparseAutoencoderCost(p, ...  % p is sae1Theta, which contains W1,W2,b1.b2
                                   inputSize, hiddenSizeL1, ...
                                   lambda, sparsityParam, ...
                                   beta, trainData), ...
                              sae1Theta, options);  % find the appropriate value of sae1Theta (=sae1OptTheta) when the cost gets its minimum value
save 'sae1OptTheta.mat' sae1OptTheta;

load sae1OptTheta.mat

 W1 = reshape(sae1OptTheta(1:inputSize * hiddenSizeL1), hiddenSizeL1, inputSize);


[sae1Features] = feedForwardAutoencoder(sae1OptTheta, hiddenSizeL1, ...
                                        inputSize, trainData);

%%======================================================================
% %% STEP 2: Train the second sparse autoencoder

%%Randomly initialize the parameters


sae2Theta = initializeParameters(hiddenSizeL2, hiddenSizeL1);

options.Method = 'lbfgs'; 
options.maxIter = 400;	
options.display = 'on';

[sae2OptTheta, cost2] = minFunc( @(p) sparseAutoencoderCost(p, ...
                                   hiddenSizeL1, hiddenSizeL2,...
                                   lambda, sparsityParam, ...
                                   beta, sae1Features), ...
                              sae2Theta, options);
save 'sae2OptTheta.mat' sae2OptTheta;

load sae2OptTheta.mat;
                      
[sae2Features] = feedForwardAutoencoder(sae2OptTheta, hiddenSizeL2, ...
                                        hiddenSizeL1, sae1Features);
                                    
 W2_0 = reshape(sae2OptTheta( 1:hiddenSizeL1 * hiddenSizeL2), hiddenSizeL2, hiddenSizeL1);


%% STEP 2.1: Train the third sparse autoencoder


% %  Randomly initialize the parameters


sae3Theta = initializeParameters(hiddenSizeL3, hiddenSizeL2);

options.Method = 'lbfgs';
options.maxIter = 400;	
options.display = 'on';

[sae3OptTheta, cost3] = minFunc( @(p) sparseAutoencoderCost(p, ...
                                   hiddenSizeL2, hiddenSizeL3,...
                                   lambda, sparsityParam, ...
                                   beta, sae2Features), ...
                              sae3Theta, options);
save 'sae3OptTheta.mat' sae3OptTheta;

load sae3OptTheta;
W3_0 = reshape(sae3OptTheta( 1:hiddenSizeL2 * hiddenSizeL3), hiddenSizeL3, hiddenSizeL2);


%%======================================================================
%% STEP 3: Train the softmax classifier

[sae3Features] = feedForwardAutoencoder(sae3OptTheta, hiddenSizeL3, ...
                                        hiddenSizeL2, sae2Features);
saeSoftmaxTheta = 0.005 * randn(hiddenSizeL3 * numClasses, 1);

[cost, grad] = softmaxCost(saeSoftmaxTheta, numClasses, hiddenSizeL3, lambda, sae3Features, trainLabels);

options.maxIter = 100;

softmaxModel = softmaxTrain(hiddenSizeL3, numClasses, lambda, ...
                            sae3Features, trainLabels, options);

saeSoftmaxOptTheta = softmaxModel.optTheta(:);

%save 'saeSoftmaxOptTheta.mat' saeSoftmaxOptTheta;

%load saeSoftmaxOptTheta;

%%======================================================================
%% STEP 5: Finetune softmax model



% Initialize the stack using the parameters learned

stack = cell(3,1);
stack{1}.w = reshape(sae1OptTheta(1:hiddenSizeL1*inputSize), ...
                     hiddenSizeL1, inputSize);
stack{1}.b = sae1OptTheta(2*hiddenSizeL1*inputSize+1:2*hiddenSizeL1*inputSize+hiddenSizeL1);

stack{2}.w = reshape(sae2OptTheta(1:hiddenSizeL2*hiddenSizeL1), ...
                     hiddenSizeL2, hiddenSizeL1);
stack{2}.b = sae2OptTheta(2*hiddenSizeL2*hiddenSizeL1+1:2*hiddenSizeL2*hiddenSizeL1+hiddenSizeL2);

stack{3}.w = reshape(sae3OptTheta(1:hiddenSizeL3*hiddenSizeL2), ...
                     hiddenSizeL3, hiddenSizeL2);
stack{3}.b = sae3OptTheta(2*hiddenSizeL3*hiddenSizeL2+1:2*hiddenSizeL3*hiddenSizeL2+hiddenSizeL3);
% Initialize the parameters for the deep model
[stackparams, netconfig] = stack2params(stack);
stackedAETheta = [ saeSoftmaxOptTheta ; stackparams ];


save '/scratch/uceedua/DL3D/mat/netconfig.mat' netconfig;



lambda = 1e-4;

addpath minFunc/
options.Method = 'lbfgs'; 
options.maxIter = 400;	 
options.display = 'on';

[stackedAEOptTheta, cost4] = minFunc( @(p) stackedAECost_multilayer(p, inputSize, hiddenSizeL3, ...
                                              numClasses, netconfig, lambda, ...
                                              trainData, trainLabels), stackedAETheta, options);
save '/scratch/uceedua/DL3D/mat/stackedAEOptTheta.mat' stackedAEOptTheta;


%load stackedAEOptTheta;

%%======================================================================
%% STEP 6: Test 

[testData,testLabels] = pretraining_testData_4(cubeDim);
%testLabels(testLabels == 0) = 10; % Remap 0 to 10

[pred,predValue1] = stackedAEPredict_multilayer(stackedAETheta, inputSize, hiddenSizeL3, ...
                          numClasses, netconfig, testData);

acc = mean(testLabels(:) == pred(:));
fprintf('Before Finetuning Test Accuracy: %0.3f%%\n', acc * 100);

[pred,predValue2] = stackedAEPredict_multilayer(stackedAEOptTheta, inputSize, hiddenSizeL3, ...
                          numClasses, netconfig, testData);
                      
predValueMean = min(predValue2(:));
fprintf('Average predValue: %0.3f %\n', predValueMean);


acc = mean(testLabels(:) == pred(:));
fprintf('After Finetuning Test Accuracy: %0.3f%%\n', acc * 100);