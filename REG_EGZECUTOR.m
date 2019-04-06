clc;
clear all;
close all;
warning off;
format short;

set(0, 'DefaultFigureRenderer', 'opengl');

global robot1;
global use_cpp_IK_solver;
global check_for_asymmetry;
global traj_set;
global traj_xyzcba;
global joint_angle_set;
global traj_grp_idx;
global port;
global type;
global dcm_obj;
global part_pts;
global traj_idx_counter;
global kdtree;
global model_ptcloud;
global model_bounds;
global centroid;
global traj_len;
global gap;
global top_face_idx;
global v f n;
global traj_type1;
global traj_type2;
global rob_motion;
global save_file;
global perform_reg;

gen_ptcloud = true;
use_cpp_IK_solver = false;
check_for_asymmetry = false;
robot1.rob_type = 'iiwa14';
traj_len = 80;
gap = 5;
port = 30005; 
type = 1;

% probing tool TCP
robot1.robot_ree_T_tee = eye(4);
robot1.robot_ree_T_tee(1:3,4) = [0; 0; 0.1049];

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
% pause(0.1);
show_origin();
view_stl_with_VF(v,f,[],[],[],[]);

% transform imported trajectory from file Button
symm_chk_bx = uicontrol(fig1,'Style','checkbox');
symm_chk_bx.Position = [1200 50 300 40];
symm_chk_bx.String = 'Check Part Symmetry';
symm_chk_bx.FontWeight = 'bold';
symm_chk_bx.ForegroundColor = [0.1,0,0.3];
symm_chk_bx.FontSize = 15;
symm_chk_bx.Callback = @(src,event)ChkPartSym(src,event);

% exit gui
dcm_obj = datacursormode(fig1);
set(dcm_obj,'SnapToDataVertex','off')
set(dcm_obj,'DisplayStyle','window');
close_win = uicontrol(fig1,'Style','pushbutton');
close_win.Position = [1650 80 150 50];
close_win.String = 'Exit';
close_win.FontWeight = 'bold';
close_win.BackgroundColor = [0.8,0.9,1];
close_win.ForegroundColor = [0.1,0,0.3];
close_win.FontSize = 12;
close_win.Callback = @CloseFigWindow;

% execute robot motion
rob_motion = uicontrol(fig1,'Style','pushbutton');
rob_motion.Position = [1650 140 150 50];
rob_motion.String = 'execute';
rob_motion.FontWeight = 'bold';
rob_motion.BackgroundColor = [0.8,0.9,1];
rob_motion.ForegroundColor = [0.1,0,0.3];
rob_motion.FontSize = 12;
rob_motion.Callback = @ExecuteMotion;

% save data to files
save_file = uicontrol(fig1,'Style','pushbutton');
save_file.Position = [1650 260 150 50];
save_file.String = 'Save data';
save_file.FontWeight = 'bold';
save_file.BackgroundColor = [0.8,0.9,1];
save_file.ForegroundColor = [0.1,0,0.3];
save_file.FontSize = 12;
save_file.Callback = @SaveData;

% perform contact based registration
perform_reg = uicontrol(fig1,'Style','pushbutton');
perform_reg.Position = [1650 200 150 50];
perform_reg.String = 'perform reg';
perform_reg.FontWeight = 'bold';
perform_reg.BackgroundColor = [0.8,0.9,1];
perform_reg.ForegroundColor = [0.1,0,0.3];
perform_reg.FontSize = 12;
perform_reg.Callback = @PerformReg;

% traj type 1 selection
traj_type1 = uicontrol(fig1,'Style','pushbutton');
traj_type1.Position = [1650 900 200 50];
traj_type1.String = '1-point method';
traj_type1.FontWeight = 'bold';
traj_type1.BackgroundColor = [0.8,0.9,1];
traj_type1.ForegroundColor = [0.1,0,0.3];
traj_type1.FontSize = 12;
traj_type1.Callback = @SelectTrajType1;

% traj type 2 selection
traj_type2 = uicontrol(fig1,'Style','pushbutton');
traj_type2.Position = [1650 840 200 50];
traj_type2.String = '2-point method';
traj_type2.FontWeight = 'bold';
traj_type2.BackgroundColor = [0.8,0.9,1];
traj_type2.ForegroundColor = [0.1,0,0.3];
traj_type2.FontSize = 12;
traj_type2.Callback = @SelectTrajType2;

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
traj_xyzcba = {};
joint_angle_set = {};
traj_grp_idx = [];
traj_idx_counter = 1;


