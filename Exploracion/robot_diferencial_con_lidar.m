%   ________   _______  _      ____  _____            _____ _____ ____  _   _ 
%  |  ____\ \ / /  __ \| |    / __ \|  __ \     /\   / ____|_   _/ __ \| \ | |
%  | |__   \ V /| |__) | |   | |  | | |__) |   /  \ | |      | || |  | |  \| |
%  |  __|   > < |  ___/| |   | |  | |  _  /   / /\ \| |      | || |  | | . ` |
%  | |____ / . \| |    | |___| |__| | | \ \  / ____ \ |____ _| || |__| | |\  |
%  |______/_/ \_\_|    |______\____/|_|  \_\/_/    \_\_____|_____\____/|_| \_|                                                                            

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

load 2022b_tp_map.mat     %carga el mapa como occupancyMap en la variable 'map'
% load mapa_2022_1c.mat   %mapa 1c2022

if verMatlab.Release=='(R2016b)'
    %Para versiones anteriores de MATLAB, puede ser necesario ajustar mapa
    imagen_mapa = 1-double(imread('2022b_tp_map.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
elseif verMatlab.Release(1:5)=='(R201'    % Completar con la version que tengan
    %Ni idea que pasa, ver si el truco R2016b funciona
    disp('ver si la compatibilidad R2016b funciona');
else
    disp(['Utilizando MATLAB ', verMatlab.Release]);
end

% CAMBIO LOS 0.5 DEL MAPA AL MAXIMO
% (https://github.com/ivolis/robot_lidar/issues/18)
map_occupancy = getOccupancy(map);
map_occupancy(map_occupancy > map.ProbabilitySaturation(1)) = map.ProbabilitySaturation(2);
setOccupancy(map,[0 0],map_occupancy)

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
initPose = [9; 15; -pi/2];         % Pose inicial (x y theta) del robot simulado (el robot pude arrancar en cualquier lugar valido del mapa)

% Inicializar vectores de tiempo, entrada y pose
tVec = 0:sampleTime:simulationDuration;         % Vector de Tiempo para duracion total

% Guardo las poses
pose = zeros(3,numel(tVec));    % Inicializar matriz de pose
pose(:,1) = initPose;

%% Simulacion

if verMatlab.Release=='(R2016b)'
    r = robotics.Rate(1/sampleTime);    %matlab viejo no tiene funcion rateControl
else
    r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente cualquier version nueva
end

%% LidarSLAM
resolution = map.Resolution;
robot = lidarSLAM(resolution, lidar.maxRange);
robot.LoopClosureThreshold = 150; % se setea empiricamente
robot.LoopClosureSearchRadius = 8; % se setea empiricamente

%% Inicializaciones

% en la primera iteracion todavia no tengo lecturas de lidar hasta el final
v_cmd = 0;
w_cmd = 0;
new_cmd = false;
% prioridades de rotacion
priority_rotations = ["left" "right"];
priority_rotation = priority_rotations(1);
% robot atrapado
trapped_routine = false;
robot_trapped = false;
tic % voy a ir chequeando cuanto tarda la simulacion asi puedo poner un corte

for idx = 2:numel(tVec)   

    %% Completado:
    
    %----------------------------------------------------------------------
    %                    Modelo para sacar w_cmd y v_cmd
    %----------------------------------------------------------------------
    if(new_cmd)
        [v_cmd, w_cmd, robot_trapped] = robot_motion(ranges, lidar.scanAngles, ...
                                              w_max, v_max, priority_rotation);
    end
    
    %----------------------------------------------------------------------
    %                           Robot atrapado
    %----------------------------------------------------------------------
    if(trapped_routine) % rutina de robot atrapado activa
        if trapped_routine_idx <= trapped_routine_length
            w_cmd = w_cmd_trapped_routine(trapped_routine_idx);
            trapped_routine_idx = trapped_routine_idx + 1;
        else
            trapped_routine = false; % termino rutina de robot atrapado
            new_cmd = true;
        end
    else
        if(robot_trapped) % esta atrapado pero no hay una rutina de RA activa
            trapped_routine = true;
            v_cmd = 0;
            w_cmd = 0; % solo por la iter que "se da cuenta" que quedo atrapado
            trapped_routine_length = randi([3 5]); % rotacion de duracion aleatoria
            if(priority_rotation == "left")
                w_cmd_trapped_routine = w_max*ones(trapped_routine_length,1);
            else
                w_cmd_trapped_routine = -w_max*ones(trapped_routine_length,1);
            end
            trapped_routine_idx = 1; % inicializacion
        end
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

    %% Completado:
    %----------------------------------------------------------------------
    %             Uso de informacion y prioridad de rotaciones
    %----------------------------------------------------------------------

    
    % Cambio la prioridad de rotacion cada tanto para evitar atascarme en
    % cualquier lugar, esto le da mas libertad al robot
    if(mod(idx,100) == 0)
        priority_rotations = flip(priority_rotations);
        priority_rotation = priority_rotations(1);
    end
    
    % Creo un scan con la lectura del lidar y los angulos asociados a c/u
    % lo hago cada tanto porque es lo mas pesado para la simulacion
    if(mod(idx,15) == 0)
        scan = lidarScan(ranges,lidar.scanAngles);
        [isScanAccepted, loopClosureInfo, optimizationInfo] = ...
                                                        addScan(robot,scan);
        if(optimizationInfo.IsPerformed)
            disp("///////LOOP CLOSURE///////")
        end
        new_cmd = true;
    end
    
    %----------------------------------------------------------------------
    %                           Visualizaciones
    %----------------------------------------------------------------------
    figure(1)
    show(robot);
    
    viz(pose(:,idx),ranges)
    waitfor(r);    
    
    % Mostrar y guardar el tiempo que tarda por cada iteracion
    toc
    
    % pasaron 3 mins ya, terminar exploracion (en version robot ver bien esto)
%     if(toc > simulationDuration) 
%         break;
%     end
    
end

%%

[scansSLAM,poses] = scansAndPoses(robot);
occMap = buildMap(scansSLAM,poses,resolution,lidar.maxRange);
figure
show(occMap)
title('Mapa de Ocupación')
xlabel('X [m]')
ylabel('Y [m]')
xlim([-1 3.3])
ylim([-2.5 2.6])
