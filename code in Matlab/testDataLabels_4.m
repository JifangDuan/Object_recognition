function [testData,testLabels] = testDataLabels_4(number_start,number_end,object,serie,cubeDim)
%to find load trainData and trainLabels for each object

testData = zeros(power(cubeDim,3),4*(number_end-number_start+1));
for i = number_start:number_end

        input_file =['C:\Users\PangTouXian\Documents\MATLAB\ModelNet10\voxel_' num2str(cubeDim) '_4/' object '/test/' object '_' num2str(i,'%04d') '_1.mat'];
        input1 = load(input_file);
        testData(:,i-number_start+1)=input1.cube1(:);

        input_file =['C:\Users\PangTouXian\Documents\MATLAB\ModelNet10/voxel_' num2str(cubeDim) '_4/' object '/test/' object '_' num2str(i,'%04d') '_2.mat'];
        input2 = load(input_file);
        testData(:,i-number_start+1+(number_end-number_start+1))=input2.cube2(:);
        
        input_file =['C:\Users\PangTouXian\Documents\MATLAB\ModelNet10/voxel_' num2str(cubeDim) '_4/' object '/test/' object '_' num2str(i,'%04d') '_3.mat'];
        input3 = load(input_file);
        testData(:,i-number_start+1+2*(number_end-number_start+1))=input3.cube3(:);
        
        input_file =['C:\Users\PangTouXian\Documents\MATLAB\ModelNet10/voxel_' num2str(cubeDim) '_4/' object '/test/' object '_' num2str(i,'%04d') '_4.mat'];
        input4 = load(input_file);
        testData(:,i-number_start+1+3*(number_end-number_start+1))=input4.cube4(:);
end


testLabels = serie*ones(4 * (number_end-number_start+1),1);

end