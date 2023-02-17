function estim_pos_robot = possible_position(particles, weights) 

    % Returns a single estimate of filter state based on the particle cloud.
    %
    % particles (M x 3): set of M particles to sample from. Each row contains a state hypothesis of dimension 3 (x, y, theta).
    % weights (M x 1): weights of the particles. Each row contains a weight.

    % initialize
    estim_pos_robot = zeros(1,3);

    %% compute mean_pos    
    
    % Se puede hacer el promedio ponderado de las particulas y sus pesos
    % para sacar la pose aproximada (en la media).
    estim_pos_robot = weights'*particles; 
    


%     queda medio raro visualmente tomar la mejor
% 
%     [max_weight, max_weight_idx] = max(weights);
%     % mejor particula
%     estim_pos_robot = particles(max_weight_idx,:);
%     
end