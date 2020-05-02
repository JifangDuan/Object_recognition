function voxel_normalise_noplot_4(file_in,cubeDim,file_out1,file_out2,file_out3,file_out4)

%%convert mesh (.off) object to voxel (.mat), each object for 4 different
%%viewing angles
%%normalise it to a certain size (cubeDim*cubeDim*cubeDim)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% file: a '.off' file
% colour: 'b' 'r'...

%cubeDim: the number of cube in each dege of the normalised volume
%(16*16*16)

% cube_size: the size (edge length) of each small cube (to be determined)
% v: vertex coordinates
% f: the three corresponding vertex serior number of each triangle
% n: number of vertex(traingle)
% meshX...: the three values of each column correspond to the X coordinate of three vertex of a triangle
% maxX...: the size of the big box
% maxx...: the size of each triangle




%%

[v,f]=voxel(file_in);
%scatter3(v(1,:),v(2,:),v(3,:));
%figure();
[m,n]=size(f);



for i=1:n
    for j=1:3
meshX(j,i)=v(1,f(j,i));
meshY(j,i)=v(2,f(j,i));
meshZ(j,i)=v(3,f(j,i));
    end
end

maxX=max(v(1,:));
minX=min(v(1,:));
maxY=max(v(2,:));
minY=min(v(2,:));
maxZ=max(v(3,:));
minZ=min(v(3,:));

%%
%normalisation: normalise to normalise 3
rangeX = maxX-minX;
rangeY = maxY-minY;
rangeZ = maxZ-minZ;
a = [rangeX,rangeY,rangeZ];
[t,I] = max(a);

if I == 1
    cube_size = rangeX/cubeDim;
    x_start = minX;
    y_start = minY + rangeX/2 - rangeY/2;
    z_start = minZ + rangeX/2 - rangeZ/2;
else
    if I == 2
        cube_size = rangeY/cubeDim;
        y_start = minY;
        x_start = minX + rangeY/2 - rangeX/2;
        z_start = minZ + rangeY/2 - rangeZ/2;
    else 
        cube_size = rangeZ/cubeDim;
        z_start = minZ;
        x_start = minX + rangeZ/2 - rangeX/2;
        y_start = minY + rangeZ/2 - rangeY/2;
    end
end







cube = zeros(cubeDim,cubeDim,cubeDim);

for i = 1:n
    maxx(i) = max(meshX(:,i));
    minx(i) = min(meshX(:,i));
    maxy(i) = max(meshY(:,i));
    miny(i) = min(meshY(:,i));
    maxz(i) = max(meshZ(:,i));
    minz(i) = min(meshZ(:,i));
end


for i = 1:n
    cube(floor((minx(i)-minX)/cube_size)+floor((x_start-minX)/cube_size)+1:ceil((maxx(i)-minX)/cube_size)+floor((x_start-minX)/cube_size),...
    floor((miny(i)-minY)/cube_size)+floor((y_start-minY)/cube_size)+1:ceil((maxy(i)-minY)/cube_size)+floor((y_start-minY)/cube_size),...
    floor((minz(i)-minZ)/cube_size)+floor((z_start-minZ)/cube_size)+1:ceil((maxz(i)-minZ)/cube_size)+floor((z_start-minZ)/cube_size)) = 1;
    
end
 cube1 = cube(1:cubeDim,1:cubeDim,1:cubeDim);

 for i = 1:cubeDim
     for j = 1:cubeDim
         for k = 1:cubeDim
             cube2(i,j,k) = cube(j,cubeDim-i+1,k);
             cube3(i,j,k) = cube(cubeDim-j+1,i,k);
             cube4(i,j,k) = cube(cubeDim-i+1,cubeDim-j+1,k);
         end
     end
 end
 
%cube_all = [cube1(:);cube2(:);cube3(:);cube4(:)];

save(file_out1,'cube1');
save(file_out2,'cube2');
save(file_out3,'cube3');
save(file_out4,'cube4');

    
    
end