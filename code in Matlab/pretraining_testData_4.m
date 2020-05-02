function [testData,testLabels] = pretraining_testData_4(cubeDim)

%bathtub
[testData_1,testLabels_1] = testDataLabels_4(107,156,'bathtub',1,cubeDim);
%bed
[testData_2,testLabels_2] = testDataLabels_4(516,615,'bed',2,cubeDim);
%chair
[testData_3,testLabels_3] = testDataLabels_4(890,989,'chair',3,cubeDim);
%desk
[testData_4,testLabels_4] = testDataLabels_4(201,286,'desk',4,cubeDim);
%dresser
[testData_5,testLabels_5] = testDataLabels_4(201,286,'dresser',5,cubeDim);
%monitor
[testData_6,testLabels_6] = testDataLabels_4(466,565,'monitor',6,cubeDim);
%night_stand
[testData_7,testLabels_7] = testDataLabels_4(201,286,'night_stand',7,cubeDim);
%sofa
[testData_8,testLabels_8] = testDataLabels_4(681,780,'sofa',8,cubeDim);
%table
[testData_9,testLabels_9] = testDataLabels_4(393,492,'table',9,cubeDim);
%toilet
[testData_10,testLabels_10] = testDataLabels_4(345,444,'toilet',10,cubeDim);


testData = [testData_1 testData_2 testData_3 testData_4 testData_5 testData_6 testData_7 testData_8 testData_9 testData_10];
testLabels = [testLabels_1;testLabels_2;testLabels_3;testLabels_4;testLabels_5;testLabels_6;testLabels_7;testLabels_8;testLabels_9;testLabels_10];



end
