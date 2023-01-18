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
initPose = [2; 2.5; -pi/2];         % Pose inicial (x y theta) del robot simulado (el robot pude arrancar en cualquier lugar valido del mapa)

% Inicializar vectores de tiempo, entrada y pose
tVec = 0:sampleTime:simulationDuration;         % Vector de Tiempo para duracion total

%% generar comandos a modo de ejemplo
% vxRef = 0.05*ones(size(tVec));   % Velocidad lineal a ser comandada
% wRef = zeros(size(tVec));       % Velocidad angular a ser comandada
% wRef(tVec < 5) = -0.2;
% wRef(tVec >=7.5) = 0.2;

pose = zeros(3,numel(tVec));    % Inicializar matriz de pose
pose(:,1) = initPose;

% Para vigilancia 
target_points = [1.5,1.3; 
                4.3, 2.1];

%% Simulacion

%##########################################################################
%                       INICIALIZACION DE PARTICULAS
%##########################################################################
n_particles = 1000
particles = initialize_particles(n_particles,map);
particles = {particles repmat(DifferentialDrive(R,L),n_particles,1)};
% % Visualizacion de particulas en mapa
% figure()
% show(map)
% hold on
% plot(particles{1}(:, 1), particles{1}(:, 2), '.');

%##########################################################################
%                  SECUENCIA DE LOCALIZACION INICIAL (WAKE UP)
%##########################################################################
wake_up_duration = 2*pi/0.5; % tiempo en dar una vuelta completa
tVec_Wake_up = 0:sampleTime:wake_up_duration;
wRef = -0.5*ones(size(tVec_Wake_up));       % Velocidad angular a ser comandada

%%
if verMatlab.Release=='(R2016b)'
    r = robotics.Rate(1/sampleTime);    %matlab viejo no tiene funcion rateControl
else
    r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente cualquier version nueva
end

for idx = 2:130%numel(tVec)   
    
    figure(2)
    show(map)
    hold on
    plot(particles{1}(:, 1), particles{1}(:, 2), '.');
    % Vuelta para localizar
    if idx <= length(tVec_Wake_up)+1
        v_cmd = 0.1;
        w_cmd = wRef(idx-1);
        % Visualizacion de particulas en mapa
        particles = motion_model([v_cmd w_cmd], particles, sampleTime);
%         particles{1}(10,:)
    else % quedarse quieto post vuelta
        v_cmd = 0;
        w_cmd = 0;
    end
    
    %% COMPLETAR ACA:
        % generar velocidades para este timestep
        
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
    
    %%
    % Aca el robot ya ejecutó las velocidades comandadas y devuelve en la
    % variable ranges la medicion del lidar para ser usada y
    % en la variable pose(:,idx) la odometría actual.
    
    %% COMPLETAR ACA:
        % hacer algo con la medicion del lidar (ranges) y con el estado
        % actual de la odometria ( pose(:,idx) )
        
        
        % Fin del COMPLETAR ACA
        
    %%
    % actualizar visualizacion
    viz(pose(:,idx),ranges)
    waitfor(r);
    

end

