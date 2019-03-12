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
for tn = 500
    
    tri = [v(f(tn,1),:);v(f(tn,2),:);v(f(tn,3),:);v(f(tn,1),:)];
    tri_safe = tri;
    scatter3d(tri,'filled');
    hold on;
    plot3d(tri,'k');
    edge_l(tri)
    hold on;
    c = mean(tri(1:3,:));
    tri = tri-tri(1,:);
    nn = n(tn,:);
    quiver3(c(1,1),c(1,2),c(1,3),nn(1,1),nn(1,2),nn(1,3),50);
    c = c - tri_safe(1,:);
    
    if nn(1,2)==0 && nn(1,3)==0
        alpha = 0;
    else
        alpha = atan(nn(1,2)/nn(1,3));
    end
    if nn(1,1)==0 && nn(1,3)==0
        beta = 0;
    else
        beta = -atan(nn(1,1)/nn(1,3));
    end
    
    p1 = tri(2,:);
    p2 = tri(3,:);
    ta = eye(4);
%     ta(1:3,1:3) = roty(beta)*rotx(alpha);
    ta(1:3,1:3) = eul2rotm([alpha,beta,0],'XYZ');
    p1t = apply_transformation(p1,ta)
    p2t = apply_transformation(p2,ta)
    nn_t = apply_transformation(nn,ta)
    c_t = apply_transformation(c,ta);
    
    hold on;
    quiver3(c_t(1,1),c_t(1,2),c_t(1,3),nn_t(1,1),nn_t(1,2),nn_t(1,3),50);

    hold on;
    tri_new = [tri(1,:);p1t;p2t;tri(1,:)];
    scatter3d(tri_new,'filled')
    hold on;
    plot3d(tri_new,'r');
    
    edge_l(tri_new)
    
    grid_pts = Generate_Grid_Points(1,1,-20,-20,20,20);
    pts = add_pts(tri_new,grid_pts);
    scatter3d(pts,'.r')
    
    p1tt = apply_transformation(p1t,inv(ta));
    p2tt = apply_transformation(p2t,inv(ta));
    ptst = apply_transformation(pts,inv(ta));
    
    tri_new2 = [tri(1,:);p1tt;p2tt;tri(1,:)];
    tri_new2 = tri_new2 + tri_safe(1,:);
    ptst2 = ptst + tri_safe(1,:);
    scatter3d(tri_new2,'filled')
    hold on;
    scatter3d(ptst2,'.k')
    hold on;
    plot3d(tri_new2,'g');
    
    
    
end

function edge_l(tri)
l1 = norm(tri(1,:)-tri(2,:));
l2 = norm(tri(1,:)-tri(3,:));
l3 = norm(tri(2,:)-tri(3,:));
end

function pts = add_pts(tri,grid_pts)

in = inpolygon(grid_pts(:,1),grid_pts(:,2),tri(:,1),tri(:,2));
loc = find(in);
store = [grid_pts(loc,1),grid_pts(loc,2)];
pts = [store,zeros(size(store,1),1)];

end






