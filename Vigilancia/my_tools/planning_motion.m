function [v_cmd_Vec, w_cmd_Vec] = planning_motion(pose,path,sampletime, v_max, w_max)

    v_cmd_Vec = [];
    w_cmd_Vec = [];
    
    for i = 1:length(path)
        % Velocidad lineal
        distance = norm(pose - path(i,:));
        t = distance/v_max; % mru -> vt=d
        v_cmd_Vec = [v_cmd_Vec v_max*ones(1,ceil(t/sampletime))];
        w_cmd_Vec = [w_cmd_Vec zeros(1,ceil(t/sampletime))];
        pose = path(i,:);
    end




end