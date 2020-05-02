function [trainData,trainLabels] = pretraining_trainData_4(cubeDim)



%bathtub
[trainData_1,trainLabels_1] = trainDataLabels_4(1,106,'bathtub',1,cubeDim);
%bed
[trainData_2,trainLabels_2] = trainDataLabels_4(1,515,'bed',2,cubeDim);
%chair
[trainData_3,trainLabels_3] = trainDataLabels_4(1,889,'chair',3,cubeDim);
%desk
[trainData_4,trainLabels_4] = trainDataLabels_4(1,200,'desk',4,cubeDim);
%dresser
[trainData_5,trainLabels_5] = trainDataLabels_4(1,200,'dresser',5,cubeDim);
%monitor
[trainData_6,trainLabels_6] = trainDataLabels_4(1,465,'monitor',6,cubeDim);
%night_stand
[trainData_7,trainLabels_7] = trainDataLabels_4(1,200,'night_stand',7,cubeDim);
%sofa
[trainData_8,trainLabels_8] = trainDataLabels_4(1,680,'sofa',8,cubeDim);
%table
[trainData_9,trainLabels_9] = trainDataLabels_4(1,392,'table',9,cubeDim);
%toilet
[trainData_10,trainLabels_10] = trainDataLabels_4(1,344,'toilet',10,cubeDim);


trainData = [trainData_1 trainData_2 trainData_3 trainData_4 trainData_5 trainData_6 trainData_7 trainData_8 trainData_9 trainData_10];
trainLabels = [trainLabels_1;trainLabels_2;trainLabels_3;trainLabels_4;trainLabels_5;trainLabels_6;trainLabels_7;trainLabels_8;trainLabels_9;trainLabels_10];



end