function traj_pts = gen_2d_traj(pt,centroid,traj_len,gap)

vec_mag = norm(centroid-pt);
traj_pts = pt;
if vec_mag~=0
    dir_vec = (centroid-pt)/vec_mag;
    d = 0;
    while d<=traj_len
        d = d+gap;
        traj_pts = [traj_pts;[pt+d*dir_vec]];
    end
end

end