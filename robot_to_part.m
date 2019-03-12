%% Compute Transformation of Points and Euler Angles from part to Robot

function [points,bx,by,bz] = robot_to_part(points,bx,by,bz)

global robot1;

T = robot1.rob_T_part;
Rdash = T;
Rdash(1:3,4) = [0;0;0];
for i=1:size(points)
    homogeneous_pt = zeros(1,4);
    homogeneous_pt(1,1:3) = points(i,:);
    homogeneous_pt(1,4) = 1;
    transformed_pt = T*homogeneous_pt(1,:)';
    pt = transformed_pt';
    points(i,:) = pt(1,1:3);
end

if size(bx,1)~=0
    for i=1:size(bx)
        homogeneous_norm = zeros(1,4);
        homogeneous_norm(1,1:3) = bx(i,:);
        homogeneous_norm(1,4) = 1;
        transformed_norm = Rdash*homogeneous_norm(1,:)';
        norm = transformed_norm';
        bx(i,:) = norm(1,1:3);
    end
end

if size(by,1)~=0
    for i=1:size(by)
        homogeneous_norm = zeros(1,4);
        homogeneous_norm(1,1:3) = by(i,:);
        homogeneous_norm(1,4) = 1;
        transformed_norm = Rdash*homogeneous_norm(1,:)';
        norm = transformed_norm';
        by(i,:) = norm(1,1:3);
    end
end

if size(bz,1)~=0
    for i=1:size(bz)
        homogeneous_norm = zeros(1,4);
        homogeneous_norm(1,1:3) = bz(i,:);
        homogeneous_norm(1,4) = 1;
        transformed_norm = Rdash*homogeneous_norm(1,:)';
        norm = transformed_norm';
        bz(i,:) = norm(1,1:3);
    end
end

end