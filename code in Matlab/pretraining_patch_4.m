%%preparing pitch data for convolution. generate patches.mat
function [patches] = pretraining_patch_4(cubeDim,patchDim,patchNumber)



[trainData_1,trainLabels_1] = trainDataLabels_4(1,106,'bathtub',1,cubeDim);
[trainData_2,trainLabels_2] = trainDataLabels_4(1,515,'bed',2,cubeDim);
[trainData_3,trainLabels_3] = trainDataLabels_4(1,889,'chair',3,cubeDim);
[trainData_4,trainLabels_4] = trainDataLabels_4(1,200,'desk',4,cubeDim);
[trainData_5,trainLabels_5] = trainDataLabels_4(1,200,'dresser',5,cubeDim);
[trainData_6,trainLabels_6] = trainDataLabels_4(1,465,'monitor',6,cubeDim);
[trainData_7,trainLabels_7] = trainDataLabels_4(1,200,'night_stand',7,cubeDim);
[trainData_8,trainLabels_8] = trainDataLabels_4(1,680,'sofa',8,cubeDim);
[trainData_9,trainLabels_9] = trainDataLabels_4(1,392,'table',9,cubeDim);
[trainData_10,trainLabels_10] = trainDataLabels_4(1,344,'toilet',10,cubeDim);

trainData = [trainData_1 trainData_2 trainData_3 trainData_4 trainData_5 trainData_6 trainData_7 trainData_8 trainData_9 trainData_10];

[testData_1,testLabels_1] = testDataLabels_4(107,156,'bathtub',1,cubeDim);
[testData_2,testLabels_2] = testDataLabels_4(516,615,'bed',2,cubeDim);
[testData_3,testLabels_3] = testDataLabels_4(890,989,'chair',3,cubeDim);
[testData_4,testLabels_4] = testDataLabels_4(201,286,'desk',4,cubeDim);
[testData_5,testLabels_5] = testDataLabels_4(201,286,'dresser',5,cubeDim);
[testData_6,testLabels_6] = testDataLabels_4(466,565,'monitor',6,cubeDim);
[testData_7,testLabels_7] = testDataLabels_4(201,286,'night_stand',7,cubeDim);
[testData_8,testLabels_8] = testDataLabels_4(681,780,'sofa',8,cubeDim);
[testData_9,testLabels_9] = testDataLabels_4(393,492,'table',9,cubeDim);
[testData_10,testLabels_10] = testDataLabels_4(345,444,'toilet',10,cubeDim);

testData = [testData_1 testData_2 testData_3 testData_4 testData_5 testData_6 testData_7 testData_8 testData_9 testData_10];

allData = [trainData testData];

n = randi([1 4899*4],patchNumber,1);
x = randi([1 cubeDim-patchDim+1],patchNumber,1);
y = randi([1 cubeDim-patchDim+1],patchNumber,1);
z = randi([1 cubeDim-patchDim+1],patchNumber,1);
patch_ref = [n,x,y,z];

for i = 1:patchNumber
    
    patch_cube = reshape(allData(:,patch_ref(i,1)),[cubeDim,cubeDim,cubeDim]);
    for ix = 1:patchDim
        for iy = 1:patchDim
            for iz = 1:patchDim
                patch_3D(ix,iy,iz) = patch_cube(x(i)+ix-1,y(i)+iy-1,z(i)+iz-1);
            end
        end
    end
              
    patches(:,i) = patch_3D(:);
   
end

end


