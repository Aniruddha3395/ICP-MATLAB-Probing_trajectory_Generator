function [pts] = Generate_Grid_Points(pathgap_x,pathgap_y,xmin,ymin,xmax,ymax)

% Function to generate the uniform mesh grid of points along the x-y plane.
% INPUT = gap between the adjacent points and maximum value
% in x and y direction.
% OUTPUT = All points consisting the uniform grid.

j = [floor(ymin):pathgap_y:ceil(ymax)]';        % iteration along y direction
i_val = [floor(xmin):pathgap_x:ceil(xmax)]';     % iteration along x direction
pts = zeros(size(j,1)*size(i_val,1),2);  
st_pt = 1;
for i = 1:size(i_val,1)
    pts(st_pt:st_pt+size(j,1)-1,:) = [i_val(i,1)*ones(size(j,1),1),j];
    st_pt = st_pt + size(j,1);
end
end