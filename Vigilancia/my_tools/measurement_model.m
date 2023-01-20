function weights = measurement_model(ranges, x, map)
%     Computes the observation likelihood of all particles.
%     
%     The employed sensor model is range only.
%     
%     ranges: lidar measures
%     x: set of current particles
%     map: map representation (binaryOccupancyMap object)
    
    sigma = 0.2;
    weights = ones(size(x, 1),1);
    angles = linspace(-pi/2,pi/2,length(ranges));
    lidar_max_range = 5;
    n_angles_to_see = 10; % c/particula vera 10 angulos distintos aleatorios del intervalo -pi/2 a pi/2
    
    if size(ranges, 1) == 0
        return
    end
    
    eta = 0; % es la normalizacion que sale de sumar los beliefs con bayes
    for i = 1:length(x) % para todas particulas
        
        idx_to_see = randperm(length(ranges),n_angles_to_see);
        angles = angles(idx_to_see);
        ranges = ranges(idx_to_see);
        intersection_points = rayIntersection(map, x(i, :), angles, lidar_max_range);

        for j = 1:length(angles) % para cada medicion
            
            if(isnan(ranges(j)) || ranges(j)< 0.20)
                % La consigna dice que si es NaN o <20 cm, es erronea
                continue;
            end
            
            if(isnan(intersection_points(j,:)))
                d_hat = lidar_max_range; % la distancia "esperada" es la maxima del sensor
            else
                d_hat = sqrt( (intersection_points(j,1) - x(i,1)).^2 + (intersection_points(j,2) - x(i,2)).^2);
            end
 
            weights(i) = weights(i) + normpdf(0,d_hat-ranges(j),sigma);% bayes
        end
        eta = eta + weights(i); % eta + belief, digamos
    
    end    
    weights = weights ./ eta; % normalizo for all x (por eso el ./)
end