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
initPose = [4; 2; -pi/2];         % Pose inicial (x y theta) del robot simulado (el robot pude arrancar en cualquier lugar valido del mapa)

% Inicializar vectores de tiempo, entrada y pose
tVec = 0:sampleTime:simulationDuration;         % Vector de Tiempo para duracion total

%% Inicializar matriz de pose
pose = zeros(3,numel(tVec));
pose(:,1) = initPose;

%% Simulacion

% Para vigilancia 
target_points = [4, 1.3;
                1.5,1.3; 
                4.3, 2.1];

%##########################################################################
%                       INICIALIZACION DE PARTICULAS
%##########################################################################
n_particles = 2000;
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


%%
if verMatlab.Release=='(R2016b)'
    r = robotics.Rate(1/sampleTime);    %matlab viejo no tiene funcion rateControl
else
    r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente cualquier version nueva
end


% Inicializo el robot en la secuencia de wakeup
sequence_state = sequence_state_wake_up;
motion_idx = 1;
path_idx = 1;
for idx = 2:numel(tVec)-1400
    
    % Visualizacion de particulas en mapa
    figure(2)
    show(map)
    hold on
    plot(particles(:, 1), particles(:, 2), '.');

    
    %% COMPLETAR ACA:
    % generar velocidades para este timestep
    
    % Vuelta para localizar
    if(strcmp(sequence_state, sequence_state_wake_up))
        v_cmd = 0;
        w_cmd = w_wake_up(idx-1);
        if idx > length(tVec_Wake_up)
            sequence_state = sequence_state_planning;
        end
    end


    
    % check que no se haya ido mucho de trayectoria y habria que ponerlo de
    % nuevo en modo planning.. o cada tanto ponerlo .
    if(strcmp(sequence_state, 'motion')) 
        v_cmd = v_motion(motion_idx);
        w_cmd = w_motion(motion_idx);
        motion_idx = motion_idx + 1;
        path_idx = path_idx + 1;
        if motion_idx >= length(v_motion)
            disp("FINALIZO RECORRIDO")
            sequence_state = 'finish';
            v_cmd = 0;
            w_cmd = 0;
        end
        plot(path(:, 1), path(:, 2), '.');
    end
    
    
    
    % fin del COMPLETAR ACA
    
    %% a partir de aca el robot real o el simulador ejecutan v_cmd y w_cmd:
    
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
    % Normalizo la orientacion de la pose
    pose(3,idx)= normalize_angle(pose(3,idx));  
   
    
    % Accion de odometria realiza segun pose actual y anterior
    odometry = get_odometry(pose(:,idx),pose(:,idx-1));
    % Muevo las particulas segun el modelo de movim
    particles = motion_model(odometry, particles);
  
    % Si sigo en wake up debo localizarme en el mapa, resampleo con el
    % modelo de medicion
    if(strcmp(sequence_state, sequence_state_wake_up))
        weights = measurement_model(ranges, particles, map);
        normalizer = sum(weights);
        weights = weights ./ normalizer;
        particles = resample(particles, weights, size(particles, 1));
    end
    
    
    if(strcmp(sequence_state, sequence_state_planning))
        estim_pos_robot_xy = possible_position(particles, weights);
        path = A_star_planning(map,estim_pos_robot_xy,target_points(1,:));
        plot(path(:, 1), path(:, 2), '.');
        [v_motion, w_motion] = planning_motion(estim_pos_robot_xy,path,sampleTime, v_max, w_max);
        sequence_state = 'motion';
    end
   

        
    % actualizar visualizacion
    viz(pose(:,idx),ranges)
    waitfor(r);
    
    
end