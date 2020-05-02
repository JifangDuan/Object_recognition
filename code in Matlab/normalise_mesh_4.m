function normalise_mesh_4(number1,number2,object,cubeDim)

for i = 1:number1
   
    file_in = ['C:\Users\PangTouXian\Documents\MATLAB\ModelNet10\ModelNet10\' object '\train\' object '_' num2str(i,'%04d') '.off'];
    file_out1 =['C:\Users\PangTouXian\Documents\MATLAB\ModelNet10\voxel_' num2str(cubeDim) '_4\' object '\train\' object '_' num2str(i,'%04d') '_1.mat'];
    file_out2 =['C:\Users\PangTouXian\Documents\MATLAB\ModelNet10\voxel_' num2str(cubeDim) '_4\' object '\train\' object '_' num2str(i,'%04d') '_2.mat'];
    file_out3 =['C:\Users\PangTouXian\Documents\MATLAB\ModelNet10\voxel_' num2str(cubeDim) '_4\' object '\train\' object '_' num2str(i,'%04d') '_3.mat'];
    file_out4 =['C:\Users\PangTouXian\Documents\MATLAB\ModelNet10\voxel_' num2str(cubeDim) '_4\' object '\train\' object '_' num2str(i,'%04d') '_4.mat'];
    
    voxel_normalise_noplot_4(file_in,cubeDim,file_out1,file_out2,file_out3,file_out4);
    
end

for i = number1+1:number2
   
    file_in = ['C:\Users\PangTouXian\Documents\MATLAB\ModelNet10\ModelNet10\' object '/test/' object '_' num2str(i,'%04d') '.off'];
    file_out1 =['C:\Users\PangTouXian\Documents\MATLAB\ModelNet10\voxel_' num2str(cubeDim) '_4/' object '/test/' object '_' num2str(i,'%04d') '_1.mat'];
    file_out2 =['C:\Users\PangTouXian\Documents\MATLAB\ModelNet10\voxel_' num2str(cubeDim) '_4/' object '/test/' object '_' num2str(i,'%04d') '_2.mat'];
    file_out3 =['C:\Users\PangTouXian\Documents\MATLAB\ModelNet10\voxel_' num2str(cubeDim) '_4/' object '/test/' object '_' num2str(i,'%04d') '_3.mat'];
    file_out4 =['C:\Users\PangTouXian\Documents\MATLAB\ModelNet10\voxel_' num2str(cubeDim) '_4/' object '/test/' object '_' num2str(i,'%04d') '_4.mat'];
    voxel_normalise_noplot_4(file_in,cubeDim,file_out1,file_out2,file_out3,file_out4);
    
end

% [v,f]=voxel(file);
% scatter3(v(1,:),v(2,:),v(3,:));
% figure();