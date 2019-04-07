function [sol_x,traj_successful] = compute_IK_2(data)

global robot1;
skip_pts = 2;
ori_options = [0 0 1]; % alignment with x - y - z vector

robot1_ree_T_tee = robot1.robot_ree_T_tee;

iiwalb = [ -168; -118; -168; -118; -168; -118; -173 ] /180.0*pi;

% % SOLVE USING fmincon % %
A = []; b = []; Aeq = []; beq = [];
fun = @single_iiwa_IK;

lb = iiwalb;
ub = -lb;
x0 = zeros(7,1);

options = optimoptions('fmincon');
options.MaxIterations = 200;
options.MaxFunctionEvaluations = 1e8;
options.StepTolerance = 1e-8;
options.OptimalityTolerance = 1e-8;
options.ObjectiveLimit = 1e-8;
% options.FiniteDifferenceStepSize = 0.25*sqrt(eps);
% options.Display = 'iter';
options.Display = 'off';

sol_x = []; xyzbxbybz = [];
failure_count = 0;
traj_successful = true;
for i = 1:skip_pts:size(data,1)
    T = [ data(i,4:6)', data(i,7:9)', data(i,10:12)', data(i,1:3)' ];
    x = fmincon(@(x)single_iiwa_IK(x,T,ori_options,robot1_ree_T_tee,x0), x0, A, b, Aeq, beq, lb, ub, [], options);
    f = single_iiwa_IK(x,T,ori_options,robot1_ree_T_tee,x0);
    if(f<0.03)
        %         disp('success');
        x0 = x;
        sol_x = [sol_x;x'];
        xyzbxbybz = [xyzbxbybz; data(i,:)];
    else
        failure_count = failure_count+1;
        if failure_count >1
            traj_successful = false;
            break;
        end
    end
end

    function f = single_iiwa_IK(x,T,ori_options,robot1_ree_T_tee,x0)
        
        position_err = 0; ori_x_err = 0; ori_y_err = 0; ori_z_err = 0;
        
        if strcmp(robot1.rob_type,'iiwa7')
            b1_T_ree = get_iiwa7_FK_all_joints_mex(x', eye(4));
            b1_T_ree = b1_T_ree(33:36,:);
        elseif strcmp(robot1.rob_type,'iiwa14')
            b1_T_ree = get_iiwa14_FK_all_joints_mex(x', eye(4));
            b1_T_ree = b1_T_ree(33:36,:);
        end
        
        b1_T_tee = b1_T_ree * robot1_ree_T_tee;
        position_err = norm(b1_T_tee(1:3,4)-T(1:3,4));
        
        if(ori_options(1)==1)
            ori_x_err = 1 - b1_T_tee(1:3,1)' * T(1:3,1);
        end
        
        if(ori_options(2)==1)
            ori_y_err = 1 - b1_T_tee(1:3,2)' * T(1:3,2);
        end
        
        if(ori_options(3)==1)
            ori_z_err = 1 - b1_T_tee(1:3,3)' * T(1:3,3);
        end
        
        time = norm(x-x0,inf);
        f = position_err + ori_x_err + ori_y_err + ori_z_err + time/(2*pi*1000);
        
    end

end