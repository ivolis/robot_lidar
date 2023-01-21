function odom = get_odometry(actual_pose,last_pose)

    diff_pose = actual_pose - last_pose;
    diff_x = diff_pose(1);
    diff_y = diff_pose(2);
    diff_angle = normalize_angle(diff_pose(3));
    
    delta_rot_1 = diff_angle/2;
    delta_trans = sqrt(diff_x * diff_x + diff_y * diff_y);
    delta_rot_2 = diff_angle - delta_rot_1;

    odom = struct('r1', delta_rot_1, 't', delta_trans, 'r2', delta_rot_2);


end