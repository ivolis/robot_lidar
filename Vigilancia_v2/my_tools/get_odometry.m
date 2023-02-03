% actual -> estado ' en bibliografia
function odom = get_odometry(actual_pose,last_pose)

    diff_pose = actual_pose(1:2) - last_pose(1:2);
    diff_x = diff_pose(1);
    diff_y = diff_pose(2);
    % angdiff(alpha,beta) == wrap2pi(beta-alpha)
    diff_angle = angdiff(last_pose(3),actual_pose(3)); 
    
    delta_rot_1 = angdiff(last_pose(3),atan2(diff_y,diff_x));
    delta_trans = sqrt(diff_x * diff_x + diff_y * diff_y);
    delta_rot_2 = angdiff(delta_rot_1, diff_angle);

    odom = struct('r1', delta_rot_1, 't', delta_trans, 'r2', delta_rot_2);


end