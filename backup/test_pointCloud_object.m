clc;
clear;
close all;

set(0, 'DefaultFigureRenderer', 'opengl');

%% adding woking directory and all dependancies
[working_dir,~,~] = fileparts(mfilename('fullpath'));
addpath(genpath(working_dir),'-end');

%% import the STL part and its corresponding pointcloud

part_name = 'test4';
% part_name = 'Dome';
% part_name = 'Heli_Blade';

stl_file_name = strcat('CAD_stl/',part_name,'.STL');
ptcloud_file_name = strcat('data_files/',part_name,'_ptcloud_mm_gap1mm.csv');

[v,f,n, ~] = stlRead(stl_file_name);
model_ptcloud = dlmread(ptcloud_file_name);

figure;
show_origin();
hold on;
view(0,0)
for tn = 100
    clc;
%     cla;
tri = [v(f(tn,1),:);v(f(tn,2),:);v(f(tn,3),:);v(f(tn,1),:)];
scatter3d(tri,'filled');
hold on;
plot3d(tri,'k');
edge_l(tri)
hold on;
tri = tri-tri(1,:);
c = mean(tri(1:3,:));
% scatter3d(tri,'filled');
% hold on;
% plot3d(tri,'b');
nn = n(tn,:);
% quiver3(c(1,1),c(1,2),c(1,3),nn(1,1),nn(1,2),nn(1,3),50);

% alpha = acos(nn(1,2));
% beta = acos(nn(1,1));
if nn(1,2)==0 && nn(1,3)==0
    alpha = 0;
else
    alpha = atan(nn(1,2)/nn(1,3))*180/pi;
end
if nn(1,1)==0 && nn(1,3)==0
    beta = 0;
else
    beta = -atan(nn(1,1)/nn(1,3))*180/pi;
end

p1 = tri(2,:);
p2 = tri(3,:);
ta = eye(4);
ta(1:3,1:3) = rotx(alpha);
tb = eye(4);
tb(1:3,1:3) = roty(beta);
p1t = apply_transformation(p1,ta);
p1t = apply_transformation(p1t,tb);
p2t = apply_transformation(p2,ta);
p2t = apply_transformation(p2t,tb);
hold on;
% scatter3d(p1t,'filled');
% scatter3d(p2t,'filled');
tri_new = [tri(1,:);p1t;p2t;tri(1,:)];
scatter3d(tri_new,'filled')
hold on;
plot3d(tri_new,'r');

edge_l(tri)

end

function edge_l(tri)

l1 = norm(tri(1,:)-tri(2,:))
l2 = norm(tri(1,:)-tri(3,:))
l3 = norm(tri(2,:)-tri(3,:))

end






