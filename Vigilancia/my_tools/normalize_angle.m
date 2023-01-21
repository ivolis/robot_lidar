% Normaliza el angulo entre -pi y pi (-180 a 180 en radianes)
function angle_norm = normalize_angle(angle)
 
    angle_norm = angle;

    angle_norm =  mod(angle_norm,2*pi); 

    % lo fuerzo a que sea el resto positivo, para que 0 <= angulo < 360  
    angle_norm = mod( (angle_norm + 2*pi), 2*pi);  
    
    % lo fuerzo en la clase de residuo de valor absoluto mínimo, tal que -180 < ángulo <= 180  
    if (angle_norm > pi)  
        angle_norm = angle_norm - 2*pi;
    end
    
    % Caso extremo
    if(angle == -pi)
        angle_norm = -pi;
    end

end