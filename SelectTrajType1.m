function SelectTrajType1(~,~)

global robot1;
global ptcloud_data;
global traj_data;
global param;

traj_data.traj_type2.Enable = 'off';
robot1.rob_motion.Enable = 'off';
param.save_file.Enable = 'off';
robot1.perform_reg.Enable = 'off';
traj_data.traj_mode = 1;

datacursormode on;

flag = 0;
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
    c_info = getCursorInfo(param.dcm_obj);
    part_pt = c_info.Position;
    ptcloud_data.part_pts = [ptcloud_data.part_pts;part_pt];
    
    %Plot the Points
    if ~isempty(ptcloud_data.part_pts)
        hold on;
        pt_scatter = scatter3(ptcloud_data.part_pts(:,1),ptcloud_data.part_pts(:,2),ptcloud_data.part_pts(:,3),50,'k','filled'); %Plot the Points
        uistack(pt_scatter,'top');
    end
    
    if ~isempty(part_pt)
        disp('model 1 calc');
        [traj_points,joint_angles,traj_flag] = generate_probe_traj(ptcloud_data.kdtree,ptcloud_data.model_ptcloud,ptcloud_data.model_bounds,...
            part_pt,ptcloud_data.centroid,traj_data.traj_len,traj_data.gap,ptcloud_data.top_face_idx,ptcloud_data.v,ptcloud_data.f,ptcloud_data.n,param.check_for_asymmetry);
        
        if traj_flag
            traj_points(:,1:3) = traj_points(:,1:3).*1000; %making values in mm
            traj_data.traj_set{end+1,1} = traj_points;
            cba = bxbybz_to_euler_mex(traj_points(:,4:6),traj_points(:,7:9),traj_points(:,10:12));
            traj_data.traj_xyzcba{end+1,1} = [traj_points(:,1:3),cba];
            traj_data.joint_angle_set{end+1,1} = joint_angles;
            traj_start_idx = traj_data.traj_idx_counter;
            traj_end_idx = traj_data.traj_idx_counter + size(traj_points,1)-1;
            traj_data.traj_grp_idx = [traj_data.traj_grp_idx;traj_start_idx,traj_end_idx];
            traj_data.traj_idx_counter = traj_data.traj_idx_counter + size(traj_points,1);
            
            hold on;
            plot3(traj_points(:,1),traj_points(:,2),traj_points(:,3),'g','LineWidth',5);
        else
            
            hold on;
            traj_points(:,1:3) = traj_points(:,1:3).*1000;
            plot3(traj_points(:,1),traj_points(:,2),traj_points(:,3),'r','LineWidth',5);
            
        end
    end
    
    break;
end

traj_data.traj_type2.Enable = 'on';
robot1.rob_motion.Enable = 'on';
param.save_file.Enable = 'on';
robot1.perform_reg.Enable = 'on';

datacursormode off;

end