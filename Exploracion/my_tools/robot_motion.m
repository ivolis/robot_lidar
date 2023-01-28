function [v_cmd, w_cmd]= robot_motion(lidar_ranges, lidar_angles, w_max, v_max)

    max_collision_distance = 0.35;
   
    % las mediciones por debajo de 20cm considero que no sirve
    idx_collisions = find( lidar_ranges > 0.2 & lidar_ranges < max_collision_distance);
    collision_angles = lidar_angles(idx_collisions);

    F = isempty(find(collision_angles < pi/4 & collision_angles > -pi/4));
    L = isempty(find(collision_angles > pi/4 & collision_angles < pi/2));
    R = isempty(find(collision_angles < -pi/4 & collision_angles > -pi/2)); % al pedo


    if F
        v_cmd = v_max;
        w_cmd = 0;
    else
        if L
            if R
                v_cmd = 0;
                % se queda trabando haciendo izq-der-izq-der y asi xd
                w_cmd = sign(round(rand(1,1))-0.5) * w_max;
            else
                v_cmd = 0;
                w_cmd = w_max;
            end
        else % R
            v_cmd = 0;
            w_cmd = -w_max;
        end
    end
    
    [F L R];

end


