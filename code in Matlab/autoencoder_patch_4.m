%% learn features from patches



patchDim = 2;
cubeDim = 16;

hiddenSize = 64;     % number of hidden units ; might be too much. maybe more layers, less neurons?????????????????
patchNumber = 40000;



sparsityParam = 0.01;   % desired average activation of the hidden units.
                     % (This was denoted by the Greek alphabet rho, which looks like a lower-case "p",
		     %  in the lecture notes). 
lambda = 0.0001;     % weight decay parameter    
beta = 3;            % weight of sparsity penalty term 


visibleSize = power(patchDim,3);   % number of input units 

patches = pretraining_patch_4(cubeDim,patchDim,patchNumber);

%  Obtain random parameters theta
theta = initializeParameters(hiddenSize, visibleSize);

%======================================================================
%%  Implement sparseAutoencoderCost
%%patches obtained from pretraining_patch.m

[cost, grad] = sparseAutoencoderCost(theta, visibleSize, hiddenSize, lambda, ...
                                     sparsityParam, beta, patches);

%%======================================================================
%% STEP 4: After verifying that your implementation of
%  sparseAutoencoderCost is correct, You can start training your sparse
%  autoencoder with minFunc (L-BFGS).

%  Randomly initialize the parameters
theta = initializeParameters(hiddenSize, visibleSize);

%  Use minFunc to minimize the function

addpath C:\Users\PangTouXian\Documents\MATLAB/DL3D/minFunc/
options.Method = 'lbfgs'; % Here, we use L-BFGS to optimize our cost
                          % function. Generally, for minFunc to work, you
                          % need a function pointer with two outputs: the
                          % function value and the gradient. In our problem,
                          % sparseAutoencoderCost.m satisfies this.
options.maxIter = 400;	  % Maximum number of iterations of L-BFGS to run 
options.display = 'on';


[opttheta2, cost] = minFunc( @(p) sparseAutoencoderCost(p, ...
                                   visibleSize, hiddenSize, ...
                                   lambda, sparsityParam, ...
                                   beta, patches), ...
                              theta, options);

save 'optTheta_patch2.mat' opttheta2;
%load optTheta_patch.mat;

%%======================================================================
%% STEP 5: Visualization 

% W1 = reshape(opttheta(1:hiddenSize*visibleSize), hiddenSize, visibleSize);
% display_network(W1', 12); 
% 
% print -djpeg weights.jpg   % save the visualization to a file 


