clc;
clear all;
close all;
warning off;
format short;

set(0, 'DefaultFigureRenderer', 'opengl');

global robot1;
global use_cpp_IK_solver;
global check_for_asymmetry;

gen_ptcloud = true;
use_cpp_IK_solver = false;
check_for_asymmetry = false;
robot1.rob_type = 'iiwa7';
traj_len = 40;
gap = 0.5;

% probing tool TCP
robot1.robot_ree_T_tee = eye(4);
robot1.robot_ree_T_tee(1:3,4) = [0; 0; 0.0968];

%% generate robot_T_part transformation

% for now, just generating transformation,
% later on, will make this service and get that transfrmation as request
robot1.rob_T_part = gen_rob_T_part();

%% adding woking directory and all dependancies
[working_dir,~,~] = fileparts(mfilename('fullpath'));
addpath(genpath(working_dir),'-end');

%% import the STL part and its corresponding pointcloud

% part_name = 'test2';
% part_name = 'Dome';
% part_name = 'Heli_Blade';
part_name = 'Composite_Mold';


% Load STL
stl_file_name = strcat('CAD_stl/',part_name,'.STL');
[v,f,n, ~] = stlRead(stl_file_name);

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

%% show part

fig1 = figure();
set(fig1,'units','normalized','outerpos',[0 0 1 1]);
axis equal;
addToolbarExplorationButtons(fig1);
show_origin();
view_stl_with_VF(v,f,[],[],[],[]);

dcm_obj = datacursormode(fig1);
set(dcm_obj,'SnapToDataVertex','off')
set(dcm_obj,'DisplayStyle','window');
close_win = uicontrol(fig1,'Style','pushbutton');
close_win.Position = [1500 40 150 50];
close_win.String = 'DONE';
close_win.FontWeight = 'bold';
close_win.BackgroundColor = [0.8,0.9,1];
close_win.ForegroundColor = [0.1,0,0.3];
close_win.FontSize = 12;
close_win.Callback = @CloseFigWindow;

% transform imported trajectory from file Button
symm_chk_bx = uicontrol(fig1,'Style','checkbox');
symm_chk_bx.Position = [50 960 250 40];
symm_chk_bx.String = 'Check Part Symmetry';
symm_chk_bx.FontWeight = 'bold';
symm_chk_bx.ForegroundColor = [0.1,0,0.3];
symm_chk_bx.FontSize = 12;
symm_chk_bx.Callback = @(src,event)ChkPartSym(src,event);


model_bounds = [min(model_ptcloud(:,1)),max(model_ptcloud(:,1));
    min(model_ptcloud(:,2)),max(model_ptcloud(:,2));
    min(model_ptcloud(:,3)),max(model_ptcloud(:,3))];
centroid = mean(model_ptcloud);
% scatter3d(centroid,'filled')

%% Filtering the faces which lie on surface only
top_face_idx = zeros(size(n,1),1);
idx_face_counter = 1;
for i=1:size(f)
    if n(i,3)>0       % positive value....equivalent to top surface
        top_face_idx(idx_face_counter,1)=i;
        idx_face_counter = idx_face_counter + 1;
    end
end
top_face_idx = top_face_idx(1:idx_face_counter-1,:);

part_pts =[];
traj_set = {};
joint_angle_set = {};
traj_grp_idx = [];
traj_idx_counter = 1;

while 1
    key=0;
    fprintf('Select Point on Model Pointcloud')
    while key==0
        try
            key = waitforbuttonpress;
        catch
            flag=1;
            break;
        end
    end
    if flag==1
        fprintf('\nSelection Complete\n');
        break;
    end
    c_info = getCursorInfo(dcm_obj);
    part_pt = c_info.Position;
    part_pts = [part_pts;part_pt];
    
    %Plot the Points
    if ~isempty(part_pts)
        hold on;
        pt_scatter = scatter3(part_pts(:,1),part_pts(:,2),part_pts(:,3),50,'k','filled'); %Plot the Points
        uistack(pt_scatter,'top');
    end
    
    if ~isempty(part_pt)
        [traj_points,joint_angles,traj_flag] = generate_probe_traj(kdtree,model_ptcloud,model_bounds,...
            part_pt,centroid,traj_len,gap,top_face_idx,v,f,n,check_for_asymmetry);
        
        if traj_flag
            traj_points(:,1:3) = traj_points(:,1:3).*1000; %making values in mm
            traj_set{end+1,1} = traj_points;
            joint_angle_set{end+1,1} = joint_angles;
            traj_start_idx = traj_idx_counter;
            traj_end_idx = traj_idx_counter + size(traj_points,1)-1;
            traj_grp_idx = [traj_grp_idx;traj_start_idx,traj_end_idx];
            traj_idx_counter = traj_idx_counter + size(traj_points,1);
            
            hold on;
            plot3(traj_points(:,1),traj_points(:,2),traj_points(:,3),'g','LineWidth',5);
        else
            
            hold on;
            traj_points(:,1:3) = traj_points(:,1:3).*1000;
            plot3(traj_points(:,1),traj_points(:,2),traj_points(:,3),'r','LineWidth',5);
            
        end
    end
end

%% write data to file

dlmwrite('data_files/probing_xyz_bxbybz.csv',cell2mat(traj_set)); % in mm
dlmwrite('data_files/probing_joint_angles.csv',cell2mat(joint_angle_set));
dlmwrite('data_files/probing_group_idx.csv',traj_grp_idx);

