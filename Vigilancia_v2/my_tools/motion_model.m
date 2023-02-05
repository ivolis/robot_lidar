% u = v_cmd , w_cmd
function new_particles = motion_model(u, particles, deltaT)
    % Samples new particle positions, based on old positions and odometry.
    %
    % u: odometry reading
    % particles: set of old particles
    
    v = u(1);
    w = u(2);
    
    % Noise parameters
    alpha = [0.1; 0.1; 0.1; 0.1; 0.01; 0.01];

    sigma_v = alpha(1)*v^2 + alpha(2)*w^2;
    sigma_w = alpha(3)*v^2 + alpha(4)*w^2;
    sigma_gamma = alpha(5)*v^2 + alpha(6)*w^2;

    v_noise = v + normrnd(0,sigma_v);  
    w_noise = w + normrnd(0,sigma_w); 
    gamma = normrnd(0,sigma_gamma); 
    
    % avoid 0 division
    w_noise(abs(w_noise)<1e-16) = 1e-16;
    R = v_noise/w_noise;

    % es un hotfix pero no tiene mucho sentido dejarlo CREO..
    % sacar con 0;%
    diversity_factor_x = normrnd(0,0.01, size(particles(:,1)));
    diversity_factor_y = normrnd(0,0.01, size(particles(:,2)));

    new_particles(:,1) = particles(:,1) - R * sin(particles(:,3)) + R * sin(particles(:,3) + w_noise * deltaT) + diversity_factor_x;
    new_particles(:,2) = particles(:,2) + R * cos(particles(:,3)) - R * cos(particles(:,3) + w_noise * deltaT) + diversity_factor_y;
    new_particles(:,3) = wrapToPi(particles(:,3) + w_noise * deltaT + gamma * deltaT);
    
end







%%% esta "funciona mejor" (Hace nube de puntos pero VER PORQUE

% function x_new = sample_motion_model(poses, x)
%     
%     diff_pose = poses(:,2) - poses(:,2-1);
%     diff_x = diff_pose(1);
%     diff_y = diff_pose(2);
%     diff_angle = wrapToPi(diff_pose(3));
%     
%     rot_1 = diff_angle /2;
%     tras = sqrt(diff_x * diff_x + diff_y * diff_y);
%     rot_2 = diff_angle /2;
% 
%     u = struct('r1', rot_1, 't', tras, 'r2', rot_2);
% 
% 
% 
%     % Samples new particle positions, based on old positions and odometry.
%     %
%     % u: odometry reading
%     % x: set of old particles
% 
%     % Noise parameters
%     noise = [0.1 0.1 0.05 0.05];
% 
%     % Particle count
%     pc = size(x, 1);
% 
%     % Compute normal distributed noise
%     O = repmat([u.r1, u.t, u.r2], pc, 1);
%     M = zeros(pc, 3);
%     S = repmat([
%         max(0.00001, noise(1) * abs(u.r1) + noise(2) * u.t) ...
%         max(0.00001, noise(3) * u.t  + noise(4) * (abs(u.r1) + abs(u.r2))) ...
%         max(0.00001, noise(1) * abs(u.r2) + noise(2) * u.t)
%     ], pc, 1);
%     N = normrnd(M, S, pc, 3);
% 
%     % Add noise to the motion for every particle
%     odom = O + N;
% 
%     % Compute new particle positions
%     x_new = x + [
%         odom(:, 2) .* cos(x(:, 3) + odom(:, 1)), ...
%         odom(:, 2) .* sin(x(:, 3) + odom(:, 1)), ...
%         odom(:, 1) + odom(:, 3)
%     ];
% 
% end

