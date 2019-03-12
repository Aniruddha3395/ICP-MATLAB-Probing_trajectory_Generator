function [points,normals] = project_2d_traj(points,top_face_idx,v,f,n)

xyz = [];
normals = [];
faces = [];
for j=1:size(top_face_idx,1)
    p1 = v(f(top_face_idx(j),1),:);
    p2 = v(f(top_face_idx(j),2),:);
    p3 = v(f(top_face_idx(j),3),:);
    triangle = [p1;p2;p3;p1];
    
    
    % Check if the all grid points lie inside any triangle for face j.
    in = inpolygon(points(:,1),points(:,2),triangle(:,1),triangle(:,2));
    % Find the row numbers of elements that actually lie inside.
    k = find(in);
    
    if ~isempty(k)
        faces = [faces;f(top_face_idx(j),:)];
        temp = [points(k,1),points(k,2)];
        normal = n(top_face_idx(j),:) / norm(n(top_face_idx(j),:));
        for norm_cnt = 1:size(k)
            normals = [normals; normal];
        end
        
        % Transfer the point to the plane
        a = ((p2(2)-p1(2))*(p3(3)-p1(3)))-((p3(2)-p1(2))*(p2(3)-p1(3)));
        b = ((p2(3)-p1(3))*(p3(1)-p1(1)))-((p3(3)-p1(3))*(p2(1)-p1(1)));
        c = ((p2(1)-p1(1))*(p3(2)-p1(2)))-((p3(1)-p1(1))*(p2(2)-p1(2)));
        d = -(a*p1(1))-(b*p1(2))-(c*p1(3));
        for count = 1:size(k)
            zval = ((-d-(a*temp(count,1))-(b*temp(count,2)))/c);
            xyz = [xyz;[temp(count,1)],[temp(count,2)],[zval]];
        end
    end
    
end

% Reorder the Points in xyz according to points-array
xyz_temp = [];
normals_temp = [];
for i=1:size(points)
    for j=1:size(xyz)
        if abs(points(i,1)-xyz(j,1))<0.001 && abs(points(i,2)-xyz(j,2))<0.001
            xyz_temp = [xyz_temp;xyz(j,:)];
            normals_temp = [normals_temp; normals(j,:)];
        end
    end
end
points = xyz_temp;
normals = normals_temp;


end