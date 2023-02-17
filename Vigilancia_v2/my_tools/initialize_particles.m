function particles = initialize_particles(count, map)
    % Returns a set of randomly initialized particles.
    particles = zeros(count,3);
    
    for i = 1:count
        potential_particle = [
            unifrnd(map.XWorldLimits(1), map.XWorldLimits(2), 1, 1), ...
            unifrnd(map.YWorldLimits(1), map.YWorldLimits(2), 1, 1), ...
            unifrnd(-pi, pi, 1, 1)
        ];
        % solo quiero particulas donde tenga sentido que este el robot
        while getOccupancy(map,[potential_particle(1), potential_particle(2)]) > map.FreeThreshold
            potential_particle = [
                unifrnd(map.XWorldLimits(1), map.XWorldLimits(2), 1, 1), ...
                unifrnd(map.YWorldLimits(1), map.YWorldLimits(2), 1, 1), ...
                unifrnd(-pi, pi, 1, 1)
            ];
        end
        particles(i,:) = potential_particle;
    end
end
            
