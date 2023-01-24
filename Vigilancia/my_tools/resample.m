function new_particles = resample(particles, weights, M)
    % Returns a new set of particles obtained by performing
    % stochastic universal sampling.
    %
    % particles (M x D): set of M particles to sample from. Each row contains a state hypothesis of dimension D.
    % weights (M x 1): weights of the particles. Each row contains a weight.
    
    new_particles = zeros(M,3);
    
    % Inicializo
    c = cumsum(weights);
    
    u = rand(1)*M^-1;
    i = 1;
    for j = 1:M
       while( u > c(i))
           i = i+1;
       end
       new_particles(j,:) = particles(i,:);
       u = u + M^-1;
    end
     
    
    
end