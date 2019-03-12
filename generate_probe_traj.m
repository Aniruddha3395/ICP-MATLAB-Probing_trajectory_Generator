function [xyz_bxbybz,joint_angles,flag] = generate_probe_traj(kdtree,model_ptcloud,model_bounds,part_pt,centroid,traj_len,gap,top_face_idx,v,f,n,check_for_asymmetry)

points = gen_2d_traj(part_pt(:,1:2),centroid(:,1:2),traj_len,gap);
[projected_points,normals] = project_2d_traj(points,top_face_idx,v,f,n);

% testing projected_points for part assymetry
if check_for_asymmetry
    asymm_flag = test_systematic_perturbation(kdtree,model_ptcloud,model_bounds,projected_points,centroid);
else
    asymm_flag = true;
end

if asymm_flag
    % testing projected points for IK
    [bx,by,bz] = compute_TCP_new(projected_points,normals);
    xyz_bxbybz = [projected_points,bx,by,bz];
    xyz_bxbybz(:,1:3) = xyz_bxbybz(:,1:3)./1000;
    [joint_angles,flag] = compute_IK_1(xyz_bxbybz); % composite style
    % [joint_angles,flag] = compute_IK_2(xyz_bxbybz); % finishing style
else
    disp('Axis/Plane symmetry exist in the part...this trajectory wont be useful...');
    xyz_bxbybz = projected_points./1000;
    joint_angles = zeros(7,1);
    flag = false;
end

end