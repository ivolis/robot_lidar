function [v_cmd, w_cmd, robot_trapped]= robot_motion(lidar_ranges, lidar_angles, w_max, v_max, priority_rotation)

    % El simulador tira NaN si no choca con nada (si el ruido esta off)
    lidar_ranges(isnan(lidar_ranges)) = 5;

    robot_trapped = false;

    max_collision_distance = 0.35*1.5;
    
    % el 0.11 es una pequeña "incertidumbre"
    alpha = wrapToPi(asin((0.35/2)/(max_collision_distance))) + 0.11;
    
    % las mediciones por debajo de 20cm considero que no sirve
    idx_collisions = find( lidar_ranges > 0.2 & lidar_ranges < max_collision_distance);
    collision_angles = lidar_angles(idx_collisions);
    
    % ~any == isempty() pero mas rapido
    F = ~any(find(collision_angles < alpha & collision_angles > -alpha));
    L = ~any(find(collision_angles > alpha & collision_angles < pi/2));
    R = ~any(find(collision_angles < -alpha & collision_angles > -pi/2));
    % F -> forward
    % L -> left       que sean true es que esa direccion esta "libre"
    % R -> right
    
    
    
    
    if F % F (priorizo ir hacia adelante principalmente)
        v_cmd = v_max;
        w_cmd = 0;
    else
        if L
            if R  
                if priority_rotation == "left" % L
                    v_cmd = 0;
                    w_cmd = w_max;
                else % R
                    v_cmd = 0;
                    w_cmd = -w_max;
                end
            else % L
                v_cmd = 0;
                w_cmd = w_max;
            end
        else % no es ni F ni L
            if R % R
                v_cmd = 0;
                w_cmd = -w_max;
            else % L F y R = 0, el robot esta "atrapado"
                v_cmd = 0;
                w_cmd = 0;
                robot_trapped = true;
            end
        end
    end
    

end


