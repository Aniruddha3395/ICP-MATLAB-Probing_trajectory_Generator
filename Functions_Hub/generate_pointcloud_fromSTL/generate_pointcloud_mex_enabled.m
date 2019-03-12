function [ptcloud,ptcloud_normals] = generate_pointcloud_mex_enabled(v,f,n,gap_x,gap_y)

% code to generate uniformly sampled data points on STL

ptcloud = zeros(1e8,3);
ptcloud_normals = zeros(1e8,3);
idx_start = 1;
for idx = 1:size(f,1)
    
    tri = [v(f(idx,1),:);v(f(idx,2),:);v(f(idx,3),:);v(f(idx,1),:)];
    nn = n(idx,:);
    tri2 = tri-tri(1,:);
    
    if nn(1,2)==0 && nn(1,3)==0
        alpha = 0;
    else
        alpha = atan(nn(1,2)/nn(1,3));
    end
    
    T = eye(4);
    T(1:3,1:3) = Rotx_mex(alpha);
    
    nn2 = Apply_Transformation_mex(nn,T);
    tri3 = Apply_Transformation_mex(tri2,T);
    
    if nn2(1,1)==0 && nn2(1,3)==0
        beta = 0;
    else
        beta = -atan(nn2(1,1)/nn2(1,3));
    end
    
    T2 = eye(4);
    T2(1:3,1:3) = Roty_mex(beta);
    
    tri4 = Apply_Transformation_mex(tri3,T2);
    grid_pts = Generate_Grid_Points_mex(gap_x,gap_y,min(tri4(:,1)),min(tri4(:,2)),max(tri4(:,1)),max(tri4(:,2)));
    
    pts = add_pts(tri4,grid_pts);
    pts2 = Apply_Transformation_mex(pts,inv(T2));
    pts3 = Apply_Transformation_mex(pts2,inv(T));
    pts3 = pts3 +  tri(1,:);
    pts3_size = size(pts3,1);
    ptcloud_idx = idx_start:pts3_size+idx_start-1;
    if pts3_size+idx_start-1 <= size(ptcloud,1)
        ptcloud(ptcloud_idx,:) = pts3;
        ptcloud_normals(ptcloud_idx,:) = ptcloud_normals(idx_start:pts3_size+idx_start-1,:) + nn;
    else
        disp('ERROR : Pointcloud size limit exceeded!');
    end
    idx_start = pts3_size+idx_start;
    
end
ptcloud = ptcloud(1:idx_start-1,:);
ptcloud_normals = ptcloud_normals(1:idx_start-1,:);

    function pts = add_pts(tri,grid_pts)
        in = inpolygon(grid_pts(:,1),grid_pts(:,2),tri(:,1),tri(:,2));
        loc = find(in);
        store = [grid_pts(loc,1),grid_pts(loc,2)];
        pts = [store,zeros(size(store,1),1)];
    end

end