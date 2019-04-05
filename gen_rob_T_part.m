function T = gen_rob_T_part()

% part_points = [571.57,571.50,13.61;
%     285.78,571.50,46.74;
%     285.78,0,46.74;
%     0,285.75,112.05
%     ];
% 
% robot_points = [227.14,-324.13,24.24;
%     521.57,-324.85,57.33;
%     519.77,248.20,56.55;
%     800.02,-35.91,123.16
%     ];

part_points = [285.78,0,46.74;
    285.78,571.50,46.74;
    571.57,571.50,13.61;
    571.57,0,13.61
    ];

robot_points = [581.79,265.69,58.80;
    597.58,-307.83,58.16;
    304.69,-313.46,25.66;
    294.82,260.04,23.55
    ];

[R,t] = point_pairs_transformation(part_points,robot_points);
T = eye(4);
T(1:3,1:3) = R;
T(1:3,4) = t;

% This function computes the Rotation and Translation matrix.
    function [R,T] = point_pairs_transformation(A,B)
        centroid_A = mean(A);
        centroid_B = mean(B);
        A = A - centroid_A;
        B = B - centroid_B;
        H = A'*B;
        [U,~,V] = svd(H);
        R = V*[1 0 0;0 1 0;0 0 det(V*U')]*U';
        if det(R)>0
            T = -R*centroid_A' + centroid_B';
        else
            fprintf('Determinant of rotation matrix is negative...')
        end
    end


end