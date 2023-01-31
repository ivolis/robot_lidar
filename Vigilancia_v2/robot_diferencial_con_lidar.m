%  __      _______ _____ _____ _               _   _  _____ _____          
%  \ \    / /_   _/ ____|_   _| |        /\   | \ | |/ ____|_   _|   /\    
%   \ \  / /  | || |  __  | | | |       /  \  |  \| | |      | |    /  \   
%    \ \/ /   | || | |_ | | | | |      / /\ \ | . ` | |      | |   / /\ \  
%     \  /   _| || |__| |_| |_| |____ / ____ \| |\  | |____ _| |_ / ____ \ 
%      \/   |_____\_____|_____|______/_/    \_\_| \_|\_____|_____/_/    \_\

%% Robot diferencial con lidar
% Robotica Movil - 2022 1c
close all
clear all
clc

verMatlab= ver('MATLAB');   % en MATLAB2020a funciona bien, ajustado para R2016b, los demas a pelearla...

simular_ruido_lidar = false; %simula datos no validos del lidar real, probar si se la banca
use_roomba=false;  % false para desarrollar usando el simulador, true para conectarse al robot real

addpath('my_tools'); % cualquier funcion extra que haya hecho yo
addpath('librobotics'); % graficar robot en particulas

%% Roomba
if use_roomba   % si se usa el robot real, se inicializa la conexion    
    rosshutdown
    pause(1)
    ipaddress_core = '192.168.0.101';
    ipaddress_local = '192.168.0.100';  %mi ip en a red TurtleNet
    setenv('ROS_IP', '192.168.0.100');
    setenv('ROS_MASTER_URI', ['http://', ipaddress_core, ':11311']);
    rosinit(ipaddress_core,11311, 'NodeHost', ipaddress_local)
    pause(1)
    laserSub = rossubscriber('/scan');
    odomSub = rossubscriber('/odom');
    cmdPub = rospublisher('/auto_cmd_vel', 'geometry_msgs/Twist');
    pause(1) % Esperar a que se registren los canales
    cmdMsg = rosmessage(cmdPub);  
end
    

%% Definicion del robot (disco de diametro = 0.35m)
R = 0.072/2;                % Radio de las ruedas [m]
L = 0.235;                  % Distancia entre ruedas [m]
dd = DifferentialDrive(R,L); % creacion del Simulador de robot diferencial

v_max = 0.15; % [m/s] abs(), el - significa ir hacia atras linealmente
w_max = 0.5; % [rad/s] abs(), + giro antihorario - giro horario

%% Creacion del entorno
load mapa_2022_1c.mat     %carga el mapa como occupancyMap en la variable 'map'

if verMatlab.Release=='(R2016b)'
    %Para versiones anteriores de MATLAB, puede ser necesario ajustar mapa
    imagen_mapa = 1-double(imread('mapa_2022_1c.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
elseif verMatlab.Release(1:5)=='(R201'    % Completar con la version que tengan
    %Ni idea que pasa, ver si el truco R2016b funciona
    disp('ver si la compatibilidad R2016b funciona');
else
    disp(['Utilizando MATLAB ', verMatlab.Release]);
end


%% Crear sensor lidar en simulador
lidar = LidarSensor;
lidar.sensorOffset = [0,0];   % Posicion del sensor en el robot (asumiendo mundo 2D)
scaleFactor = 3;                %decimar lecturas de lidar acelera el algoritmo
num_scans = 513/scaleFactor;
hokuyo_step_a = deg2rad(-90);
hokuyo_step_c = deg2rad(90);

lidar.scanAngles = linspace(hokuyo_step_a,hokuyo_step_c,num_scans);
lidar.maxRange = 5;

%% Crear visualizacion
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Parametros de la Simulacion

simulationDuration = 3*60;          % Duracion total [s]
sampleTime = 0.1;                   % Sample time [s]
initPose = [2; 2; -pi/2];         % Pose inicial (x y theta) del robot simulado (el robot pude arrancar en cualquier lugar valido del mapa)

% Inicializar vectores de tiempo, entrada y pose
tVec = 0:sampleTime:simulationDuration;         % Vector de Tiempo para duracion total

% Guardo las poses
pose = zeros(3,numel(tVec)); % Inicializar matriz de pose
pose(:,1) = initPose;

%% Simulacion

% Para vigilancia 
target_points = [1.5, 1.3;
                 4.3, 2.1];

%##########################################################################
%                       INICIALIZACION DE PARTICULAS
%##########################################################################
n_particles = 2^12;
particles = initialize_particles(n_particles,map);

%##########################################################################
%                  SECUENCIA DE LOCALIZACION INICIAL (WAKE UP)
%##########################################################################
wake_up_duration = 2*pi/w_max; % tiempo en dar una vuelta completa a w_max = 0.5
tVec_Wake_up = 0:sampleTime:wake_up_duration;
w_wake_up = -w_max*ones(size(tVec_Wake_up));
sequence_state_wake_up = 'wakeup';

%##########################################################################
%                  SECUENCIA DE PLANEAMIENTO (PLANNING)
%##########################################################################
sequence_state_planning = 'planning';
inflated_map = copy(map);
inflate(inflated_map,0.3);

%%
if verMatlab.Release=='(R2016b)'
    r = robotics.Rate(1/sampleTime);    %matlab viejo no tiene funcion rateControl
else
    r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente cualquier version nueva
end


%% Inicialiciones del robot
sequence_state = sequence_state_wake_up; % inicia en secuencia wake up
cmd_motion_idx = 1;
path_idx = 1;
new_cmds = false;
resample_idx = 1;
resample_iter_ease = 4;
target_point_idx = 1;

tic % cuento cuanto la simulacion
for idx = 2:numel(tVec)
    
    %% Completado:
    
    %----------------------------------------------------------------------
    %  Visualizacion de particulas en mapa (Quitar en version para robot)
    %----------------------------------------------------------------------
    figure(2)
    show(map)
    hold on
    scatter(particles(:, 1), particles(:, 2), '.', 'blue')

    
    %----------------------------------------------------------------------
    %                  Vuelta para localizarse
    %----------------------------------------------------------------------
    if(strcmp(sequence_state, sequence_state_wake_up))
        v_cmd = 0;
        w_cmd = w_wake_up(idx-1);
        if idx > length(tVec_Wake_up)
            sequence_state = sequence_state_planning;
        end
    end
    
    %----------------------------------------------------------------------
    %  Pedir nuevos comandos para movimiento a lo largo de la trayectoria
    %----------------------------------------------------------------------
    if(new_cmds)
        if(path_idx <= length(path))
            estim_pos_robot = possible_position(particles, weights);
            [v_cmd_motion, w_cmd_motion, close_flag] = get_cmd(estim_pos_robot, ...
                                                    path(path_idx,:),  ...
                                                    sampleTime, v_max, w_max);
            new_cmds = false;
            % Visualizacion del path (Quitar en version para robot)
            plot(path(:, 1), path(:, 2), '.');
            % los movimientos angulares no avazan el path
            if(v_cmd_motion ~= 0  | close_flag == true)
                path_idx = path_idx + 1;
            end
        else
            % En caso de terminar la trayectoria, me quedo quieto.
            if(target_point_idx == size(target_points, 1))
                sequence_state = 'finish';
                v_cmd = 0;
                w_cmd = 0;
                new_cmds = false;
            else
                % Llegue a un punto pero no al destino final, espero 3 segs
                target_point_idx = target_point_idx + 1;
                sequence_state = 'quiet';
                v_cmd_motion = zeros(1,ceil(3/sampleTime));
                w_cmd_motion = zeros(1,ceil(3/sampleTime));
                path_idx = 1;
                new_cmds = false;
            end
        end
    end
    
    %----------------------------------------------------------------------
    %         Acampado del robot (3 segundos) en puntos intermedios
    %----------------------------------------------------------------------
    if(strcmp(sequence_state, 'quiet')) 
        if cmd_motion_idx <= length(v_cmd_motion) % pongo v pero puede ser w
            v_cmd = v_cmd_motion(cmd_motion_idx);
            w_cmd = w_cmd_motion(cmd_motion_idx);
            cmd_motion_idx = cmd_motion_idx + 1;
        else
            new_cmds = true;
            cmd_motion_idx = 1; 
            sequence_state = sequence_state_planning;
        end
        % Visualizo particulas y pose aprox robot (Quitar en version para robot)
        scatter(target_points(target_point_idx,1), ...
                    target_points(target_point_idx,2), 'p' , 'red');
        drawrobot(possible_position(particles, weights), 'r', 2, 0.35, 0.35);
    end
    
    %----------------------------------------------------------------------
    %               Movimiento a lo largo de la trayectoria
    %----------------------------------------------------------------------
    if(strcmp(sequence_state, 'motion')) 
        if cmd_motion_idx <= length(v_cmd_motion) % pongo v pero puede ser w
            v_cmd = v_cmd_motion(cmd_motion_idx);
            w_cmd = w_cmd_motion(cmd_motion_idx);
            cmd_motion_idx = cmd_motion_idx + 1;
        else
            new_cmds = true;
            cmd_motion_idx = 1;
            % testeo cada cuanto usar el meas model
            weights = measurement_model(ranges, particles, map);
            normalizer = sum(weights);
            weights = weights ./ normalizer;
            particles = resample(particles, weights, size(particles, 1));   
        end
        % Visualizo particulas,  trayectoria y pose aprox robot (Quitar en version para robot)
        scatter(path(:, 1), path(:, 2), '.', 'black');
        scatter(target_points(target_point_idx,1), ...
                 target_points(target_point_idx,2), 'p' , 'magenta');
        drawrobot(possible_position(particles, weights), 'r', 2, 0.35, 0.35);
    end
    

    
    %%
    %----------------------------------------------------------------------
    %                 EJECUCION DE V_CMD Y W_CMD (No tocar)
    %----------------------------------------------------------------------
    
    if use_roomba       % para usar con el robot real
        
        % Enviar comando de velocidad
        cmdMsg.Linear.X = v_cmd;
        cmdMsg.Angular.Z = w_cmd;
        send(cmdPub,cmdMsg);
        
        % Recibir datos de lidar y odometría
        scanMsg = receive(laserSub);
        odompose = odomSub.LatestMessage;
        
        % Obtener vector de distancias del lidar
        ranges_full = laserSub.LatestMessage.Ranges;
        ranges = ranges_full(1:scaleFactor:end);
        %ranges = circshift(ranges,length(ranges)/2);  % verificar
        ranges(ranges==0)=NaN; % lecturas erroneas y maxrange
        % Obtener pose del robot [x,y,yaw] de datos de odometría (integrado por encoders).
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose(:,idx) = [odompose.Pose.Pose.Position.X + initPose(1); odompose.Pose.Pose.Position.Y+ initPose(2); odomRotation(1)];
    
    else        % para usar el simulador
   
        % Mover el robot segun los comandos generados
        [wL,wR] = inverseKinematics(dd,v_cmd,w_cmd);
        % Velocidad resultante
        [v,w] = forwardKinematics(dd,wL,wR);
        velB = [v;0;w]; % velocidades en la terna del robot [vx;vy;w]
        vel = bodyToWorld(velB,pose(:,idx-1));  % Conversion de la terna del robot a la global
        % Realizar un paso de integracion
        pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
        % Tomar nueva medicion del lidar
        ranges = lidar(pose(:,idx));
        if simular_ruido_lidar
            % Simular ruido de un lidar ruidoso (probar a ver si se la banca)
            chance_de_medicion_no_valida = 0.17;
            not_valid=rand(length(ranges),1);
            ranges(not_valid<=chance_de_medicion_no_valida)=NaN;
        end
    end
    %% Completado
    %----------------------------------------------------------------------
    %        Accion de odometria realiza segun pose actual y anterior
    %----------------------------------------------------------------------
    odometry = get_odometry(pose(:,idx),pose(:,idx-1));
    % Muevo las particulas segun el modelo de movimiento por odometria
    particles = motion_model(odometry, particles);
    
    %----------------------------------------------------------------------
    %     Modelo de medicion y resampleo respecto a la rutina wake up
    %----------------------------------------------------------------------
    % Si sigo en wake up debo localizarme en el mapa, resampleo con el
    % modelo de medicion y voy bajando la cantidad para alivianar computo
    if(strcmp(sequence_state, sequence_state_wake_up))
        if(resample_idx < resample_iter_ease)
            weights = measurement_model(ranges, particles, map);
            normalizer = sum(weights);
            weights = weights ./ normalizer;
            particles = resample(particles, weights, size(particles, 1)/2);
            resample_idx = resample_idx + 1;
        else
            weights = measurement_model(ranges, particles, map);
            normalizer = sum(weights);
            weights = weights ./ normalizer;
            particles = resample(particles, weights, size(particles, 1));
        end
    end
    
    %----------------------------------------------------------------------
    %  Una vez localizado hay que planear la trayectoria (algoritmo A*)
    %----------------------------------------------------------------------
    if(strcmp(sequence_state, sequence_state_planning))
        estim_pos_robot = possible_position(particles, weights);
        path = A_star_planning(inflated_map,estim_pos_robot(1:2), ...
                                target_points(target_point_idx,:));
        sequence_state = 'motion'; % una vez planeado, hay que moverse
        new_cmds = true; % pido comandos
    end
    
    %----------------------------------------------------------------------
    %         Actualizar visualizacion (Quitar en version para robot)
    %----------------------------------------------------------------------
    viz(pose(:,idx),ranges)
    waitfor(r);

    % Dibujo el robot parado en la "meta" (Quitar en version para robot)
    if(strcmp(sequence_state, 'finish'))
        drawrobot(possible_position(particles, weights), 'r', 2, 0.35, 0.35);
    end

    toc % veo cuanto tardo hasta esta iteracion
    
end