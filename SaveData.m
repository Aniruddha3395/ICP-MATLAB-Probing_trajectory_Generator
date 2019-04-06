function SaveData(~,~)

global traj_data;

dlmwrite('/home/cl/Documents/KUKA_Shared/probing_xyz_bxbybz.csv',cell2mat(traj_data.traj_set)); % in mm
dlmwrite('/home/cl/Documents/KUKA_Shared/probing_xyz_cba.csv',cell2mat(traj_data.traj_xyzcba)); % in mm
dlmwrite('/home/cl/Documents/KUKA_Shared/probing_joint_angles.csv',cell2mat(traj_data.joint_angle_set));
dlmwrite('/home/cl/Documents/KUKA_Shared/probing_group_idx.csv',traj_data.traj_grp_idx);

end