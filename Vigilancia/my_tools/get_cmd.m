function [v_cmd_Vec, w_cmd_Vec, close_flag] = get_cmd(pose,path_point,sampletime, v_max, w_max)

    [angle_between_points, distance] = cart2pol(path_point(1) - pose(1), ...
                                            path_point(2) - pose(2));
    if distance < 0.25 % esto es mucho, encararlo por otro lado (idx de path?)
        v_cmd_Vec = 0;
        w_cmd_Vec = 0;
        close_flag = true;
    else
        close_flag = false;
        angle_diff = normalize_angle(angle_between_points-pose(3));
        if abs(angle_diff) > 0.2
            % Velocidad angular
            t = abs(angle_diff)/w_max; % mcu -> wt = theta;
            w_max = sign(angle_diff)*w_max;
            v_cmd_Vec = v_max*zeros(1,floor(t/sampletime));
            w_cmd_Vec = w_max*ones(1,floor(t/sampletime));
        else
            % Velocidad lineal
            t = distance/v_max; % mru -> vt=d
            v_cmd_Vec = v_max*ones(1,ceil(t/sampletime));
            w_cmd_Vec = w_max*zeros(1,ceil(t/sampletime));
        end
    end
    


end