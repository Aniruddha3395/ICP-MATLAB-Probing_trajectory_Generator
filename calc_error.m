function E = calc_error(kdtree,model_ptcloud,traj_pts)

num_of_neighbours = 3;
E = zeros(size(traj_pts,1),num_of_neighbours);
% E = 0;
neighbour_idx = knnsearch(kdtree,traj_pts,'K',num_of_neighbours);
for i=1:num_of_neighbours
E(:,i) = dist(model_ptcloud(neighbour_idx(:,1),:),traj_pts);
end

% mean error
E = mean(mean(E,2),1);

% rms error
% E = rms(rms(E,2),1);

end