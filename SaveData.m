function SaveData(~,~)

global traj_data;

dlmwrite('/data_files/probing_xyz_bxbybz.csv',cell2mat(traj_data.traj_set)); % in mm
dlmwrite('/data_files/probing_xyz_cba.csv',cell2mat(traj_data.traj_xyzcba)); % in mm
dlmwrite('/data_files/probing_joint_angles.csv',cell2mat(traj_data.joint_angle_set));
dlmwrite('/data_files/probing_group_idx.csv',traj_data.traj_grp_idx);

end