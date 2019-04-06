function SelectTrajType2(~,~)

global dcm_obj;
global part_pts;
global traj_idx_counter;
global traj_set;
global traj_xyzcba;
global joint_angle_set;
global traj_grp_idx;
global kdtree;
global model_ptcloud;
global model_bounds;
global centroid;
global traj_len;
global gap;
global top_face_idx;
global v f n;
global check_for_asymmetry;
global traj_type1;
global rob_motion;
global save_file;
global perform_reg;

traj_type1.Enable = 'off';
rob_motion.Enable = 'off';
save_file.Enable = 'off';
perform_reg.Enable = 'off';

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
        c_info = getCursorInfo(dcm_obj);
        part_pt = c_info.Position;
        part_pts = [part_pts;part_pt];
        
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
            disp('model 2 calc');
            [traj_points,joint_angles,traj_flag] = generate_probe_traj(kdtree,model_ptcloud,model_bounds,...
                part_pt,centroid,traj_len,gap,top_face_idx,v,f,n,check_for_asymmetry);
            
            if traj_flag
                traj_points(:,1:3) = traj_points(:,1:3).*1000; %making values in mm
                traj_set{end+1,1} = traj_points;
                cba = bxbybz_to_euler_mex(traj_points(:,4:6),traj_points(:,7:9),traj_points(:,10:12));
                traj_xyzcba{end+1,1} = [traj_points(:,1:3),cba];
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
    break;        
end

traj_type1.Enable = 'on';
rob_motion.Enable = 'on';
save_file.Enable = 'on';
perform_reg.Enable = 'on';


end