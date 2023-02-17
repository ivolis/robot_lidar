function [v_cmd, w_cmd, in_position] = ...
            get_cmd(pose,path_point, v_max, w_max, distance_thr,angle_thr)

    K = 0.6;
                                        
    euclidean_distance =  norm(path_point(1:2)- pose(1:2));
    angle_goal = atan2(path_point(2)-pose(2), path_point(1)-pose(1));
    angle_diff = angdiff(pose(3),angle_goal);

    if(euclidean_distance > distance_thr)
        in_position = false;
        if(abs(angle_diff) > angle_thr)
            
            v_cmd = 0;
            w_cmd = K*angle_diff;
            if(w_cmd > w_max)
                w_cmd = sign(angle_diff)*w_max;
            end
        else
            v_cmd = v_max;
            w_cmd = 0;
        end
    else
        in_position = true;
        v_cmd = 0;
        w_cmd = 0;
    end


end