function SaveData(~,~)

global traj_set;
global traj_xyzcba;
global joint_angle_set;
global traj_grp_idx;

dlmwrite('/home/cl/Documents/KUKA_Shared/probing_xyz_bxbybz.csv',cell2mat(traj_set)); % in mm
dlmwrite('/home/cl/Documents/KUKA_Shared/probing_xyz_cba.csv',cell2mat(traj_xyzcba)); % in mm
dlmwrite('/home/cl/Documents/KUKA_Shared/probing_joint_angles.csv',cell2mat(joint_angle_set));
dlmwrite('/home/cl/Documents/KUKA_Shared/probing_group_idx.csv',traj_grp_idx);

end