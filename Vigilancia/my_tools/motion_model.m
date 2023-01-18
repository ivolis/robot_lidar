% u = v_cmd , w_cmd
function x_new = motion_model(u, x, sampleTime)

    v_cmd = u(1);
    w_cmd = u(2);
    for i = 1:length(x{1,1}(:,1))
        
        dd = x{2}(i);
        
        % Mover el robot segun los comandos generados
        [wL,wR] = inverseKinematics(dd,v_cmd,w_cmd);
        % Velocidad resultante
        [v,w] = forwardKinematics(dd,wL,wR);
        velB = [v;0;w]; % velocidades en la terna del robot [vx;vy;w]
        vel = bodyToWorld(velB,x{1}(i,:));  % Conversion de la terna del robot a la global
        % Realizar un paso de integracion
        x_new{1}(i,:) = x{1}(i,:) + (vel*sampleTime)';
        x_new{2}(i) = x{2}(i); % esto deja visto que no hace falta el vector de dd xd
    end

end