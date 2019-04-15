clc;
clear all;
close all;
warning off;
format short;

set(0, 'DefaultFigureRenderer', 'opengl');

global robot1;
global ptcloud_data;
global traj_data;
global param;

gen_ptcloud = true;
param.use_cpp_IK_solver = false;
param.check_for_asymmetry = false;
robot1.rob_type = 'iiwa14';
traj_data.traj_len = 80;
traj_data.gap = 5;
param.port = 30007; 
param.type = 1;
traj_data.constraint_j4 = true;

% probing tool TCP
robot1.robot_ree_T_tee = eye(4);
robot1.robot_ree_T_tee(1:3,4) = [0; 0; 0.1049];

%% generate robot_T_part transformation

% for now, just generating transformation,
% later on, will make this service and get that transfrmation as request

% composite mold
robot1.rob_T_part = gen_rob_T_part();

% % for Heli Blade
% robot1.rob_T_part = eye(4);
% robot1.rob_T_part(1,4) = 381;
% robot1.rob_T_part(2,4) = 736;
% robot1.rob_T_part(1:3,1:3) = rotz(180);

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
[ptcloud_data.v,ptcloud_data.f,ptcloud_data.n, ~] = stlRead(stl_file_name);

% Apply transformation to STL
ptcloud_data.v = apply_transformation(ptcloud_data.v, robot1.rob_T_part);
T_n = robot1.rob_T_part;
T_n(1:3,4)=[0;0;0]; 
ptcloud_data.n = apply_transformation(ptcloud_data.n, T_n);

% Generate or load pointcloud
if gen_ptcloud
    gap_x = 1;
    gap_y = 1;
    [ptcloud_data.model_ptcloud,model_ptcloud_normals] = generate_pointcloud_mex_enabled(ptcloud_data.v,ptcloud_data.f,ptcloud_data.n,gap_x,gap_y);
else
    load(strcat('data_files/',part_name,'_ptcloud_mm_gap1mm.mat'));
    ptcloud_data.model_ptcloud = pts_set;
    model_ptcloud_normals = normals;
end

% making kdtree for part pointcloud
ptcloud_data.kdtree = KDTreeSearcher(ptcloud_data.model_ptcloud);

%% show part

fig1 = figure();
set(fig1,'units','normalized','outerpos',[0 0 1 1]);
axis equal;
addToolbarExplorationButtons(fig1);
% pause(0.1);
show_origin();
view_stl_with_VF(ptcloud_data.v,ptcloud_data.f,[],[],[],[]);

% transform imported trajectory from file Button
symm_chk_bx = uicontrol(fig1,'Style','checkbox');
symm_chk_bx.Position = [1200 50 300 40];
symm_chk_bx.String = 'Check Part Symmetry';
symm_chk_bx.FontWeight = 'bold';
symm_chk_bx.ForegroundColor = [0.1,0,0.3];
symm_chk_bx.FontSize = 15;
symm_chk_bx.Callback = @(src,event)ChkPartSym(src,event);

% exit gui
param.dcm_obj = datacursormode(fig1);
set(param.dcm_obj,'SnapToDataVertex','off')
set(param.dcm_obj,'DisplayStyle','window');
close_win = uicontrol(fig1,'Style','pushbutton');
close_win.Position = [1650 80 150 50];
close_win.String = 'Exit';
close_win.FontWeight = 'bold';
close_win.BackgroundColor = [0.8,0.9,1];
close_win.ForegroundColor = [0.1,0,0.3];
close_win.FontSize = 12;
close_win.Callback = @CloseFigWindow;

% execute robot motion
robot1.rob_motion = uicontrol(fig1,'Style','pushbutton');
robot1.rob_motion.Position = [1650 200 150 50];
robot1.rob_motion.String = 'execute';
robot1.rob_motion.FontWeight = 'bold';
robot1.rob_motion.BackgroundColor = [0.8,0.9,1];
robot1.rob_motion.ForegroundColor = [0.1,0,0.3];
robot1.rob_motion.FontSize = 12;
robot1.rob_motion.Callback = @ExecuteMotion;

% save data to files
param.save_file = uicontrol(fig1,'Style','pushbutton');
param.save_file.Position = [1650 260 150 50];
param.save_file.String = 'Save data';
param.save_file.FontWeight = 'bold';
param.save_file.BackgroundColor = [0.8,0.9,1];
param.save_file.ForegroundColor = [0.1,0,0.3];
param.save_file.FontSize = 12;
param.save_file.Callback = @SaveData;

% perform contact based registration
robot1.perform_reg = uicontrol(fig1,'Style','pushbutton');
robot1.perform_reg.Position = [1650 140 150 50];
robot1.perform_reg.String = 'perform reg';
robot1.perform_reg.FontWeight = 'bold';
robot1.perform_reg.BackgroundColor = [0.8,0.9,1];
robot1.perform_reg.ForegroundColor = [0.1,0,0.3];
robot1.perform_reg.FontSize = 12;
robot1.perform_reg.Callback = @PerformReg;

% traj type 1 selection
traj_data.traj_type1 = uicontrol(fig1,'Style','pushbutton');
traj_data.traj_type1.Position = [1650 900 200 50];
traj_data.traj_type1.String = '1-point method';
traj_data.traj_type1.FontWeight = 'bold';
traj_data.traj_type1.BackgroundColor = [0.8,0.9,1];
traj_data.traj_type1.ForegroundColor = [0.1,0,0.3];
traj_data.traj_type1.FontSize = 12;
traj_data.traj_type1.Callback = @SelectTrajType1;

% traj type 2 selection
traj_data.traj_type2 = uicontrol(fig1,'Style','pushbutton');
traj_data.traj_type2.Position = [1650 840 200 50];
traj_data.traj_type2.String = '2-point method';
traj_data.traj_type2.FontWeight = 'bold';
traj_data.traj_type2.BackgroundColor = [0.8,0.9,1];
traj_data.traj_type2.ForegroundColor = [0.1,0,0.3];
traj_data.traj_type2.FontSize = 12;
traj_data.traj_type2.Callback = @SelectTrajType2;

ptcloud_data.model_bounds = [min(ptcloud_data.model_ptcloud(:,1)),max(ptcloud_data.model_ptcloud(:,1));
    min(ptcloud_data.model_ptcloud(:,2)),max(ptcloud_data.model_ptcloud(:,2));
    min(ptcloud_data.model_ptcloud(:,3)),max(ptcloud_data.model_ptcloud(:,3))];
ptcloud_data.centroid = mean(ptcloud_data.model_ptcloud);
% scatter3d(ptcloud_data.centroid,'filled')

%% Filtering the faces which lie on surface only

ptcloud_data.top_face_idx = zeros(size(ptcloud_data.n,1),1);
idx_face_counter = 1;
for i=1:size(ptcloud_data.f)
    if ptcloud_data.n(i,3)>0       % positive value....equivalent to top surface
        ptcloud_data.top_face_idx(idx_face_counter,1)=i;
        idx_face_counter = idx_face_counter + 1;
    end
end
ptcloud_data.top_face_idx = ptcloud_data.top_face_idx(1:idx_face_counter-1,:);

ptcloud_data.part_pts =[];
traj_data.traj_set = {};
traj_data.traj_xyzcba = {};
traj_data.joint_angle_set = {};
traj_data.traj_grp_idx = [];
traj_data.traj_idx_counter = 1;


