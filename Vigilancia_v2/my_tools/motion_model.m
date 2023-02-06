% u = v_cmd , w_cmd
function new_particles = motion_model(u, particles, deltaT)
    % Samples new particle positions, based on old positions and odometry.
    %
    % u: odometry reading
    % particles: set of old particles
    
    v = u(1);
    w = u(2);
    
    % Noise parameters
    alpha = [0.13; 0.13; 0.13; 0.13; 0.06; 0.06];

    sigma_v = alpha(1)*v^2 + alpha(2)*w^2;
    sigma_w = alpha(3)*v^2 + alpha(4)*w^2;
    sigma_gamma = alpha(5)*v^2 + alpha(6)*w^2;

    % aca esta el tema, se le aplica el mismo ruido a todas las particulas
    % vectorizar esto
    v_noise = v + normrnd(0,sigma_v, [length(particles) 1]);
    w_noise = w + normrnd(0,sigma_w, [length(particles) 1]); 
    gamma = normrnd(0,sigma_gamma, [length(particles) 1]); 
    
    % avoid 0 division
    w_noise(abs(w_noise)<1e-16) = 1e-16;
    R = v_noise/w_noise;


    new_particles(:,1) = particles(:,1) - R * sin(particles(:,3)) + R * sin(particles(:,3) + w_noise * deltaT);
    new_particles(:,2) = particles(:,2) + R * cos(particles(:,3)) - R * cos(particles(:,3) + w_noise * deltaT);
    new_particles(:,3) = wrapToPi(particles(:,3) + w_noise * deltaT + gamma * deltaT);
    
end