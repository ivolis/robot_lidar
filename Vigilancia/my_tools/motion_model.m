% u = v_cmd , w_cmd
function x_new = motion_model(u, x, sampleTime, diffdrive)
    
    x_new = zeros(size(x));
    v_cmd = u(1);
    w_cmd = u(2);
    
    for i = 1:length(x(:,1))
        
        dd = diffdrive;
        
        % Mover el robot segun los comandos generados
        [wL,wR] = inverseKinematics(dd,v_cmd,w_cmd);
        % Velocidad resultante
        [v,w] = forwardKinematics(dd,wL,wR);
        velB = [v;0;w]; % velocidades en la terna del robot [vx;vy;w]
        vel = bodyToWorld(velB,x(i,:));  % Conversion de la terna del robot a la global
        % Realizar un paso de integracion
        x_new(i,:) = x(i,:) + (vel*sampleTime)';
    end

end
