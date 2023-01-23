function estim_pos_robot_xy = possible_position(particles, weights) 

    [max_weight, max_weight_idx] = max(weights);
    best_particle = particles(max_weight_idx,:);
    estim_pos_robot_xy = [best_particle(1) best_particle(2)];
    
end