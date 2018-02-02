clc
clear all 
close all

silhouettes = cell(8,1);

srcFiles = dir('C:\Users\Jolton\Desktop\HW4\*.pbm');  % the folder in which ur images exists
for i = 1 : length(srcFiles)
    filename = strcat('C:\Users\Jolton\Desktop\HW4\',srcFiles(i).name);
    silhouettes{i} = im2double(imread(filename));
%     figure, imshow(silhouettes);
end

Image = imread('C:\Users\Jolton\Desktop\HW4\cam07_00023_0000008550.png');

% Images = cell(8,1);

% srcFiles = dir('C:\Users\Jolton\Desktop\HW4\*.png');  % the folder in which ur images exists
% for i = 1 : length(srcFiles)
%     filename = strcat('C:\Users\Jolton\Desktop\HW4\',srcFiles(i).name);
%     Images{i} = imread(filename);
% %     figure, imshow(silhouettes);
% end

rawP = [ 776.649963  -298.408539 -32.048386  993.1581875 132.852554  120.885834  -759.210876 1982.174000 0.744869  0.662592  -0.078377 4.629312012;
    431.503540  586.251892  -137.094040 1982.053375 23.799522   1.964373    -657.832764 1725.253500 -0.321776 0.869462  -0.374826 5.538025391;
    -153.607925 722.067139  -127.204468 2182.4950   141.564346  74.195686   -637.070984 1551.185125 -0.769772 0.354474  -0.530847 4.737782227;
    -823.909119 55.557896   -82.577644  2498.20825  -31.429972  42.725830   -777.534546 2083.363250 -0.484634 -0.807611 -0.335998 4.934550781;
    -715.434998 -351.073730 -147.460815 1978.534875 29.429260   -2.156084   -779.121704 2028.892750 0.030776  -0.941587 -0.335361 4.141203125;
    -417.221649 -700.318726 -27.361042  1599.565000 111.925537  -169.101776 -752.020142 1982.983750 0.542421  -0.837170 -0.070180 3.929336426;
    94.934860   -668.213623 -331.895508 769.8633125 -549.403137 -58.174614  -342.555359 1286.971000 0.196630  -0.136065 -0.970991 3.574729736;
    452.159027  -658.943909 -279.703522 883.495000  -262.442566 1.231108    -751.532349 1884.149625 0.776201  0.215114  -0.592653 4.235517090];


Proj_mat = zeros(3,4,8);

for i=1:8
    for j=1:3
        Proj_mat(j,1:4,i) = rawP(i,4*(j-1)+1:4*(j-1)+4);
    end
end

%% Grid dimensions

x_len = 2.5 -(-2.5);
y_len = 3 -(-3);
z_len = 2.5 - 0;

grid_vol = x_len*y_len*z_len;

no_of_voxels = 50000000;

voxel_vol = grid_vol/no_of_voxels;

voxel_len = nthroot(voxel_vol,3);

img_cord = zeros(3,1,8);

three_D_voxel_count = 0;

% x_y_vec = zeros(1,6);
x_y_vec = []; Color = [];
l = 0;

for x = -2.5:voxel_len:2.5
    for y = -3:voxel_len:3
        for z = 0:voxel_len:2.5
            count = 0;
            world_cord = [x; y; z; 1];
            for i = 1:8
                img_cord(:,:,i) = Proj_mat(:,:,i)*world_cord;
                img_cord(:,:,i) = round(img_cord(:,:,i)/img_cord(3,:,i));
                if img_cord(2,:,i) > size(silhouettes{i},1) || img_cord(2,:,i) <= 0 || img_cord(1,:,i) > size(silhouettes{i},2) || img_cord(1,:,i) <= 0
                    continue;
                elseif silhouettes{i}(img_cord(2,:,i),img_cord(1,:,i)) == 1
                    count = count + 1;
                    if count == 8
                        l = l+1;
                        three_D_voxel_count = three_D_voxel_count+1;
                        world_cord = reshape(world_cord(1:3), [1,3]);
                        x_y_vec(l,1:3) = world_cord;
                        Color(l,1:3) = reshape(Image(img_cord(2,:,i),img_cord(1,:,i),1:3), [1,3]);
%                         Color(l,1:3) = reshape(Images{randi([1 8],1,1)}(img_cord(2,:,i),img_cord(1,:,i),1:3), [1,3]);
                        count = 0;
                    end
                elseif silhouettes{i}(img_cord(2,:,i),img_cord(1,:,i)) == 0
                    break;
                end
            end
        end
    end
end


% write_ply('Dancer.ply', x_y_vec, Color)

ptCloud = pointCloud(x_y_vec);
ptCloud.Color = uint8(Color);
pcwrite(ptCloud,'Ballerina','PLYFormat','ascii')
show = pcread('Ballerina.ply');
pcshow(show)

