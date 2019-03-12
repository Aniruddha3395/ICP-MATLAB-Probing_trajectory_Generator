function flag = test_systematic_perturbation(kdtree,model_ptcloud,model_bounds,traj_pts,centroid)

flag = false;

Err_threshold = 1;

% fixed values
% perturb_x = (model_bounds(1,2)-model_bounds(1,1))*0.5;
% perturb_y = (model_bounds(2,2)-model_bounds(2,1))*0.5;
% perturb_z = (model_bounds(3,2)-model_bounds(3,1))*0.5;
% perturb_Rz = 90; 
% perturb_Ry = 90;
% perturb_Rx = 90;

% random values
tot_count = 20;
success = 0;
for i = 1:tot_count
perturb_x = model_bounds(1,1) + (model_bounds(1,2)-model_bounds(1,1))*randi([50 80],1,1)/100;
perturb_y = model_bounds(2,1) + (model_bounds(2,2)-model_bounds(2,1))*randi([50 80],1,1)/100;
perturb_z = model_bounds(3,1) + (model_bounds(3,2)-model_bounds(3,1))*randi([50 80],1,1)/100;
perturb_Rz = -180 + 360*randi([90 100],1,1)/100;
perturb_Ry = -180 + 360*randi([90 100],1,1)/100;
perturb_Rx = -180 + 360*randi([90 100],1,1)/100;

% Error Value without any perturbation
E_start = calc_error(kdtree,model_ptcloud,traj_pts);

% Error Value after x-translation
traj_pts_x_shift = traj_pts;
traj_pts_x_shift(:,1) = traj_pts_x_shift(:,1) - perturb_x;
% scatter3d(traj_pts_x_shift,'.b')
E_x_shift = calc_error(kdtree,model_ptcloud,traj_pts_x_shift);
if abs(E_start-E_x_shift)>Err_threshold
    % Error Value after y-translation
    traj_pts_y_shift = traj_pts;
    traj_pts_y_shift(:,2) = traj_pts_y_shift(:,2) - perturb_y;
%     scatter3d(traj_pts_y_shift,'.b')
    E_y_shift = calc_error(kdtree,model_ptcloud,traj_pts_y_shift);
    if abs(E_start-E_y_shift)>Err_threshold
        % Error Value after z-translation
        traj_pts_z_shift = traj_pts;
        traj_pts_z_shift(:,3) = traj_pts_z_shift(:,3) - perturb_z;
%         scatter3d(traj_pts_z_shift,'.b')
        E_z_shift = calc_error(kdtree,model_ptcloud,traj_pts_z_shift);
        if abs(E_start-E_z_shift)>Err_threshold
            % Error Value after Rx-rotation about centroid
            T = eye(4);
            T(1:3,1:3) = rotx(perturb_Rx);
            traj_pts_Rx_shift = traj_pts - centroid;
            traj_pts_Rx_shift = apply_transformation(traj_pts_Rx_shift,inv(T));
            traj_pts_Rx_shift = traj_pts_Rx_shift + centroid;
%             scatter3d(traj_pts_Rx_shift,'.b')
            E_Rx_shift = calc_error(kdtree,model_ptcloud,traj_pts_Rx_shift);
            if abs(E_start-E_Rx_shift)>Err_threshold
                % Error Value after Ry-rotation about centroid
                T = eye(4);
                T(1:3,1:3) = roty(perturb_Ry);
                traj_pts_Ry_shift = traj_pts - centroid;
                traj_pts_Ry_shift = apply_transformation(traj_pts_Ry_shift,inv(T));
                traj_pts_Ry_shift = traj_pts_Ry_shift + centroid;
%                 scatter3d(traj_pts_Ry_shift,'.b')
                E_Ry_shift = calc_error(kdtree,model_ptcloud,traj_pts_Ry_shift);
                if abs(E_start-E_Ry_shift)>Err_threshold
                    % Error Value after Rz-rotation about centroid
                    T = eye(4);
                    T(1:3,1:3) = rotz(perturb_Rz);
                    traj_pts_Rz_shift = traj_pts - centroid;
                    traj_pts_Rz_shift = apply_transformation(traj_pts_Rz_shift, inv(T));
                    traj_pts_Rz_shift = traj_pts_Rz_shift + centroid;
%                     scatter3d(traj_pts_Rz_shift,'.b')
                    E_Rz_shift = calc_error(kdtree,model_ptcloud,traj_pts_Rz_shift);
                    if abs(E_start-E_Rz_shift)>Err_threshold
%                         disp('No symmetry exist')
                        success = success + 1;
                    else
%                         disp('part symmetry about Z axis exist');
                    end
                else
%                     disp('part symmetry about Y axis exist');
                end
            else
%                 disp('part symmetry about X axis exist');
            end
        else
%             disp('part symmetry along Z axis exist');
        end
    else
%         disp('part symmetry along Y axis exist');
    end
else
%     disp('part symmetry along X axis exist');
end

end

success_rate = success*100/tot_count;
if success_rate>80
    flag = true;
end
end
