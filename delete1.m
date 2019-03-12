clc;
clear all;
close all;

global robot1;

gen_ptcloud = true;

%% generate robot_T_part transformation
% for now, just generating transformation,
% later on, will make this service and get that transfrmation as request
robot1.rob_T_part = gen_rob_T_part();

%% adding woking directory and all dependancies
[working_dir,~,~] = fileparts(mfilename('fullpath'));
addpath(genpath(working_dir),'-end');

%% import the STL part and its corresponding pointcloud
part_name = 'test4';
% part_name = 'Dome';
% part_name = 'Heli_Blade';
% part_name = 'Composite_Mold';


% Load STL
stl_file_name = strcat('CAD_stl/',part_name,'.STL');
[v,f,n, ~] = stlRead(stl_file_name);

data = dlmread('data_files/probing_xyz_bxbybz.csv');

% Apply transformation to STL
v = apply_transformation(v, robot1.rob_T_part);
T_n = robot1.rob_T_part;
T_n(1:3,4)=[0;0;0]; 
n = apply_transformation(n, T_n);


% Generate or load pointcloud
if gen_ptcloud
    gap_x = 1;
    gap_y = 1;
    [model_ptcloud,model_ptcloud_normals] = generate_pointcloud_mex_enabled(v,f,n,gap_x,gap_y);
else
    load(strcat('data_files/',part_name,'_ptcloud_mm_gap1mm.mat'));
    model_ptcloud = pts_set;
    model_ptcloud_normals = normals;
end

% making kdtree for part pointcloud
kdtree = KDTreeSearcher(model_ptcloud);

model_bounds = [min(model_ptcloud(:,1)),max(model_ptcloud(:,1));
    min(model_ptcloud(:,2)),max(model_ptcloud(:,2));
    min(model_ptcloud(:,3)),max(model_ptcloud(:,3))];
centroid = mean(model_ptcloud);

figure;
show_origin();
hold on;
scatter3d(model_ptcloud,'.g');
hold on;
scatter3(centroid(1),centroid(2),centroid(3),'k','filled');
hold on;

traj_pts = data(:,1:3);

% perturb Rx;
theta = 50;
T = eye(4);
T(1:3,1:3) = rotx(50);
traj_T = traj_pts - centroid;
traj_T = apply_transformation(traj_T,inv(T));
traj_T = traj_T + centroid;
scatter3d(traj_T,'.r');
hold on;
scatter3(traj_pts(:,1),traj_pts(:,2),traj_pts(:,3),'b','filled');

traj_pts = data(:,1:3);
E1 = calc_error(kdtree,model_ptcloud,traj_pts)

E2 = calc_error(kdtree,model_ptcloud,traj_T)


% flag1 = test_perturbation(kdtree,model_ptcloud,model_bounds,projected_points,centroid);