clc;
clear;
close all;
set(0, 'DefaultFigureRenderer', 'opengl');

%% adding woking directory and all dependancies
[working_dir,~,~] = fileparts(mfilename('fullpath'));
addpath(genpath(working_dir),'-end');

%% import the STL part and its corresponding pointcloud

part_name = 'test2';
% part_name = 'Dome';
% part_name = 'Composite_Mold';
% part_name = 'Heli_Blade';

stl_file_name = strcat('CAD_stl/',part_name,'.STL');
ptcloud_file_name = strcat('data_files/',part_name,'_ptcloud_mm_gap1mm.csv');

[v,f,n, ~] = stlRead(stl_file_name);
model_ptcloud = dlmread(ptcloud_file_name);

pts_set = [];

for tn = 1:size(f,1)

disp(tn);
tri = [v(f(tn,1),:);v(f(tn,2),:);v(f(tn,3),:);v(f(tn,1),:)];
nn = n(tn,:);
c = mean(tri(1:3,:));
tri2 = tri-tri(1,:);
c2 = c - tri(1,:);
if nn(1,2)==0 && nn(1,3)==0
    alpha = 0;
else
    alpha = atan(nn(1,2)/nn(1,3));
end
T = eye(4);
T(1:3,1:3) = rotx(alpha*180/pi);
nn2 = apply_transformation(nn,T);
tri3 = apply_transformation(tri2,T);
if nn2(1,1)==0 && nn2(1,3)==0
    beta = 0;
else
    beta = -atan(nn2(1,1)/nn2(1,3));
end
T2 = eye(4);
T2(1:3,1:3) = roty(beta*180/pi);
nn3 = apply_transformation(nn2,T2);
tri4 = apply_transformation(tri3,T2);
grid_pts = Generate_Grid_Points(1,1,min(tri4(:,1)),min(tri4(:,2)),max(tri4(:,1)),max(tri4(:,2)));
pts = add_pts(tri4,grid_pts);
pts2 = apply_transformation(pts,inv(T2));
pts3 = apply_transformation(pts2,inv(T));
pts3 = pts3 +  tri(1,:);
pts_set = [pts_set;pts3];

end

figure;
view_stl_with_VF(v,f,[],[],[],[]);
hold on;
scatter3d(pts_set,'.');

function pts = add_pts(tri,grid_pts)

in = inpolygon(grid_pts(:,1),grid_pts(:,2),tri(:,1),tri(:,2));
loc = find(in);
store = [grid_pts(loc,1),grid_pts(loc,2)];
pts = [store,zeros(size(store,1),1)];

end






