function estim_pos_robot = possible_position(particles, weights) 

    [max_weight, max_weight_idx] = max(weights);
    % mejor particula
    estim_pos_robot = particles(max_weight_idx,:);
    
end