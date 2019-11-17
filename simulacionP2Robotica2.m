function simulacionP2Robotica2()
    %% Conexi?n con el simulador
    % Se inicializa la conexi?n al simulador
    disp('Program started');
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
    % Se verifica si fue posible establecer la conexi?n con el simulador 
    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);
    % Nos aseguramos que se cierre la conexi?n cuando el script se vea
    % interrumpido
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
    % Se inicia la simulaci?n
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
    %% Definición de las funciones auxiliares para manejar el gripper
    % open_gripper() abre el gripper mientras que close_gripper() lo cierra
    open_gripper = @() vrep.simxSetIntegerSignal(id, 'RG2_open', 1, ...
        vrep.simx_opmode_oneshot);
    close_gripper = @() vrep.simxSetIntegerSignal(id, 'RG2_open', 0, ...
        vrep.simx_opmode_oneshot); 
    %% Obtencion de handles y de posiciones de mesas(metas)
    % Se define un struct que contendr? todos los handles de los objetos
    % en la escena de V-REP
    h = struct('id', id);  
    % Se obtiene el handle del sensor de vision
    [~, h.cam] = vrep.simxGetObjectHandle(id, 'Vision_sensor', vrep.simx_opmode_oneshot_wait); 
    % Se obtiene el handle del chasis del robot (para encontrar su
    % posici?n)
    [~, h.robotchasis] = vrep.simxGetObjectHandle(id, 'Pioneer_p3dx_visible', vrep.simx_opmode_oneshot_wait); 
    % Se obtienen los handles de los motores del robot
    [~, h.leftmotor] = vrep.simxGetObjectHandle(id, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait);
    [~, h.rightmotor] = vrep.simxGetObjectHandle(id, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait);
    %timestep utilizado en la simulacion de VREP
    timestep = 0.05;
    %Se obtiene imagen a color por medio de visionsensor    
    color_image = 0;
    [num1,array1,color_image]= vrep.simxGetVisionSensorImage2(id,h.cam,0,vrep.simx_opmode_oneshot_wait);
    %imshow(color_image)
    %Se hace un threshold de la imagen para obtener una imagen BW donde
    %solo se observan las mesas a las que se desea llegar
    [BW1,maskedRGBImage1] = MaskGoal(color_image);
    %Se encuentran los centroides de las mesas
    s=regionprops('table',BW1,'Centroid','MinorAxisLength','MajorAxisLength','FilledArea' );
    centers = s.Centroid
%                 %Se crea un checkerboard de medida 5, ya que en VREP se tiene que cada
%                 %tile es de 5, y se tiene un tamaño total de 20*20
%                 IC = checkerboard(5);
%                 %Se almacenan las variables obtenidas de cpselect en el workspace y se
%                 %llaman 
%                 fixedPointsF=evalin('base','fixedPoints1');
%                 movingPointsF=evalin('base','movingPoints1');
%                 %Se ingresa la imagen con la cual se realizara la comparacion para
%                 %encontrar el centroide
%                 mapaC=imread('DefautlCam.jpg');
%                 %cpselect(mapaC,IC);
%                 tform = fitgeotrans(movingPointsF, fixedPointsF, 'projective');
%                 %Se realiza un warp del mapa utilizado y de la transformacion realizada
%                 J=imwarp(mapaC, tform);
%                 %imshow(J)
%                 %Se realiza una mascara para poder obtener los centroides de las metas
%                 %deseadas
%                 [BWMetas,maskedRGBImage2] = MaskGoal(J);
%                 %Se encuentran los centroides y otras propiedades
%                 sMap=regionprops('table',BWMetas, 'centroid', 'MinorAxisLength','MajorAxisLength')
%                 %%%%%%%%%%%%%%%%%%%%%%%NOTA%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Para 2g se tiene el centroide en (8,7), y al restarle 11.2 y 11.02 se
    %llega a la posicion en vrep.
    %Para 3g se tiene el centroide en (9,5), y al restarle 10.37 y 10.63 se
    %llega a la posicion en vrep.

    %Por estos resultados se realizo otro metodo para encontrar las metas y la
    %posicion de los objetos a tomar, ya que esto afecta el resultado final de
    %la simulacion, pero se ejemplifica arriba el proceso por el cual se deben
    %obtener
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Se cambia a otro vision sensor   
    %[~, h.cam] = vrep.simxGetObjectHandle(id, 'Vision_sensor0', vrep.simx_opmode_oneshot_wait); 
    [~, h.cam] = vrep.simxGetObjectHandle(id, 'Vision_sensor0', vrep.simx_opmode_oneshot_wait); 
    color_image1 = 0;
    [num2,array2,color_image1]= vrep.simxGetVisionSensorImage2(id,h.cam,0,vrep.simx_opmode_oneshot_wait);
    %imshow(color_image1);
    %Se hace un threshold de la imagen para obtener una imagen BW donde
    %solo se observan los objetos a las que se desea llegar
    [BW2,maskedRGBImage2] = MaskGoal(color_image1);
    %Se encuentran los centroides de los objetos
    s1=regionprops('table',BW2,'Centroid','MinorAxisLength','MajorAxisLength' );
    centers1 = s1.Centroid;
    %Se cambia a otro vision sensor
    [~, h.cam] = vrep.simxGetObjectHandle(id, 'Vision_sensor', vrep.simx_opmode_oneshot_wait); 
    %Se inicializan variables de posiciones
    m1x=0;
    m1y=0;
    m2x=0;
    m2y=0;
    m3x=0;
    m3y=0;
    m4x=0;
    m4y=0;
    m5x=0;
    m5y=0;
    %Se mapean los centroides encontrados a coordenadas de vrep, utilizando
    %como parametros para la ecuacion la cantidad de pixeles utilizados por
    %la camara y la distancia que lee el visionsensor (cuadro azul 20x20).
    %Se realiza un ajuste final a las variables ya mapeadas para que el
    %robot movil no choque con la mesa
    %Mesa1
    m1x=centers(1,1);
    m1y=centers(1,2);
    m1x=(m1x-128.5)/12.8;
    m1y=(m1y-145)/(34.28);
    m1x=m1x+0.35;
    m1xO=centers(5,1);
    m1yO=centers(5,2);
    m1xOF=(m1xO-128.5)/12.8;
    m1yOF=(m1yO-145)/(34.28);
    %Mesa2
    m2x=centers(2,1);
    m2y=centers(2,2);
    m2x=(m2x-128.5)/12.8;
    m2y=(m2y-145)/(-16.89);
    m2y=m2y-0.5;
    m2xO=centers(2,1);
    m2yO=centers(2,2);
    m2xOF=(m2xO-128.5)/12.8;
    m2yOF=(m2yO-145)/(-16.89);
    %Mesa3
    m3x=centers(3,1);
    m3y=centers(3,2);
    m3x=(m3x-128.5)/12.8;
    m3y=((m3y+10)-145)/(-15.73);
    m3y=m3y-0.4;
    m3xO=centers(3,1);
    m3yO=centers(3,2);
    m3xOF=(m3xO-128.5)/12.8;
    m3yOF=((m3yO+10)-145)/(-15.73);
    %Mesa4
    m4x=centers(4,1);
    m4y=centers(4,2);
    m4x=(m4x-128.5)/12.8;
    m4y=(m4y-145)/(-17.46);
    m4y=m4y-0.5;
    m4xO=centers(4,1);
    m4yO=centers(4,2);
    m4xOF=(m4xO-128.5)/12.8;
    m4yOF=(m4yO-145)/(-17.46);
    %Mesa5
    m5x=centers(5,1);
    m5x=m5x+10;
    m5y=centers(5,2);
    m5x=(m5x-128.5)/12.8;
    m5y=(m5y-145)/(7.05);
    m5y=m5y-0.5;
    m5xO=centers(5,1);
    m5yO=centers(5,2);
    m5xOF=(m5xO-128.5)/12.8;
    m5yOF=(m5yO-145)/(7.05);
    %Se crea un array con las posiciones, intercalando una posiciones
    %previamente obtenida del lugar al que debe llegar el Pioneer para
    %obtener los objetos
    Posiciones_Metas = [m5x,m5y,-0.3,-1.6750,m4x,m4y,-0.4805,-1.3,m3x,m3y,-0.4805,-1.3,m2x,m2y,-0.4805,-1.3,m1x,m1y];
    Iteracion=0;
%% Ciclo para repetir movimientos del brazo y la creacion de rutas cambiando de posiciones  
    while(Iteracion ~= 9)    

        if(Iteracion==0)||(Iteracion==2)||(Iteracion==4)||(Iteracion==6)||(Iteracion==8)   
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% Se realiza el movimiento inicial del brazo (recoger el objeto y llegar a una posicion final que permita el recorrido)
            open_gripper = @() vrep.simxSetIntegerSignal(id, 'RG2_open', 1, ...
            vrep.simx_opmode_oneshot);
            close_gripper = @() vrep.simxSetIntegerSignal(id, 'RG2_open', 0, ...
            vrep.simx_opmode_oneshot);
            % Se obtienen los handles de las juntas del UR3
            armJoints = - ones(1, 6);
            for i = 1:6
            [~, armJoints(i)] = vrep.simxGetObjectHandle(id, sprintf('UR3_joint%d', i), vrep.simx_opmode_oneshot_wait); 
            end
            h.armJoints = armJoints;
            % Se obtiene el handler del UR3
            [~, h.armBase] = vrep.simxGetObjectHandle(id, 'UR3_link1_visible', vrep.simx_opmode_oneshot_wait);
            %Se define la pose en la que inicia el UR3
            IK0 = [0; 0; 0; 0; 0; 0];
            vrep.simxPauseCommunication(id, true);  
            for i = 1:6
                vrep.simxSetJointTargetPosition(id, h.armJoints(i), IK0(i), vrep.simx_opmode_oneshot);
            end
            vrep.simxPauseCommunication(id, false); 
            pause(2);
            %Se obtienen las posiciones de los objetos a los que se desea llegar
            %Objeto1
            pcx1=(128.5-centers1(1,1))/(-80);
            pcy1=(145-centers1(1,2))/(33);
            %Objeto2
            pcx2=(128.5-centers1(2,1))/(-92.75);
            pcy2=(145-centers1(2,2))/(-42.93);
            %Objeto3
            pcx3=(128.5-centers1(3,1))/(54.28);
            pcy3=(145-centers1(3,2))/(-6.44);
            %Se definen los vectores de posicion de los objetos
            ObjetoI1=[pcx1,pcy1,0.43];
            ObjetoI2=[pcx2,pcy2,0.46];
            ObjetoI3=[pcx3,pcy3,0.43];
            ObjetoInercialV=0;
            %Se selecciona la posicion a la que se desea llegar dependiendo de la
            %iteracion en la que se encuentra el programa
            if(Iteracion==0)
            ObjetoInercialV=ObjetoI1;
            offsetIn=[0,0,0.07];
            end
            if(Iteracion==2)
            ObjetoInercialV=ObjetoI2;
            offsetIn=[0,0,0.07];
            end
            if(Iteracion==4)
            ObjetoInercialV=ObjetoI3;
            end
            %Se realiza la traspuesta del vector de posicion
            ObjetoInercialT = ObjetoInercialV'+offsetIn';
            RotPos = rpy2r(double(0),double(0),double(0),'xyz');
            InercialTObjeto = [RotPos, ObjetoInercialT; 0,0,0,1];
            %Se obtiene la posicion del robot respecto al marco inercial
            [~, poser] = vrep.simxGetObjectPosition(id, h.armBase,-1, vrep.simx_opmode_oneshot_wait);
            [~, trr] = vrep.simxGetObjectOrientation(id, h.armBase, -1, vrep.simx_opmode_oneshot_wait);
            RBrazoUR3 = rpy2r(double(trr(1)),double(trr(2)),double(trr(3)),'xyz');
            %T del robot respecto al inercial
            InercialTRobot=[RBrazoUR3, poser'; 0,0,0,1];
            RobotTInercial=inv(InercialTRobot);
            F=RobotTInercial*InercialTObjeto;
            TMesa=F;
            %Se realiza la matriz de pose final del brazo UR3
            tMesa=[0.5,0,1]';
            RdMeta = [1 0 0;0 1 0; 0 0 1];
            TFinal = [RdMeta, tMesa; 0,0,0,1];
            %Se realiza la matriz de pose intermedia del brazo UR3 
            tPoseMedia=[0.3;0;0.6];
            TMedio=[RotPos,tPoseMedia;0,0,0,1];
            %Se realiza la cinematica inversa de las poses que se encontraron
            IK1=ur3IK(TMesa,IK0,1)';
            IK2=ur3IK(TMedio,IK1',1)';
            IK3=ur3IK(TFinal,IK2',1)';
            %Se realizan trayectorias que cumplen con las poses que se obtuvieron
            Trj1=mtraj(@lspb, IK0',IK1,200);
            Trj2=mtraj(@lspb, IK1,IK2,200);
            Trj3=mtraj(@lspb, IK2,IK3,200);
            Q=[Trj1;Trj2;Trj3];
            %Se realiza el movimiento del UR3
            k=1;
            open_gripper();
            while(k < 601)
                tic; 
                q=Q(k,:);
                if(k==200)
                   close_gripper(); 
                   pause(3);
                end
                vrep.simxPauseCommunication(id, true);  
                for i = 1:6
                    vrep.simxSetJointTargetPosition(id, h.armJoints(i), q(i), vrep.simx_opmode_oneshot);
                end
                vrep.simxPauseCommunication(id, false);
                k = k + 1;
                % Se verifica que se encuentre sincronizado con la velocidad de
                % VREP
                elapsed = toc;
                timeleft = timestep - elapsed;
                if timeleft > 0
                    pause(min(timeleft, .01));
                end
            end
        end
        %% Generacion de Trayectorias
        dt = timestep;
        t0 = 0; % Tiempo inicial
        tf = 160; % Tiempo final
        k = 0; 
        K = (tf-t0)/dt; % Numero total de iteraciones 
        %Se obtiene la posicion del (objeto) chasis del robot
        PosChasis = zeros(1,3);
        [~, PosChasis(1,1:3)]= vrep.simxGetObjectPosition(id, h.robotchasis,-1,vrep.simx_opmode_oneshot_wait);
        posinicialch = zeros(1,2);
        posinicialch(1,1:2)=PosChasis(1,1:2);
        %Se crea una imagen por medio del vision sensor para crear el mapa
        I = 0;
        [num,array,I]= vrep.simxGetVisionSensorImage2(id,h.cam,1,vrep.simx_opmode_oneshot_wait);
        imshow(I);
        %Se realiza un threshold para que no se detecten las mesas a las que se
        %desean llegar
        map = I>201;
        %Se hace un flip de la imagen para que corresponda a las coordenadas
        %del mapa en VREP
        map = flip(map,1);
        map = double(map);
        %Se utiliza el algoritmo D* para crear los recorridos que debe seguir
        %el Pioneer
        %Se utiliza un inflate para que el robot no choque con obstaculos
        ds=Dstar(map,'inflate',8);
        %Se compara la iteracion actual para poder seleccionar las coordenadas
        %de la posicion final del Pioneer
        if(Iteracion == 0)     
            PosMetaX=Posiciones_Metas(1);
            PosMetaY= Posiciones_Metas(2);
        end  
        if(Iteracion == 1)     
            PosMetaX=Posiciones_Metas(3);
            PosMetaY= Posiciones_Metas(4);
        end   
        if(Iteracion == 2)     
            PosMetaX=Posiciones_Metas(5);
            PosMetaY= Posiciones_Metas(6);
        end  
        if(Iteracion == 3)     
            PosMetaX=Posiciones_Metas(7);
            PosMetaY= Posiciones_Metas(8);
        end  
        if(Iteracion == 4)     
            PosMetaX=Posiciones_Metas(9);
            PosMetaY= Posiciones_Metas(10);
        end  
            if(Iteracion == 5)     
            PosMetaX=Posiciones_Metas(11);
            PosMetaY= Posiciones_Metas(12);
        end  
        if(Iteracion == 6)     
            PosMetaX=Posiciones_Metas(13);
            PosMetaY= Posiciones_Metas(14);
        end   
        if(Iteracion == 7)     
            PosMetaX=Posiciones_Metas(15);
            PosMetaY= Posiciones_Metas(16);
        end  
        if(Iteracion == 8)     
            PosMetaX=Posiciones_Metas(17);
            PosMetaY= Posiciones_Metas(18);
        end  
        %Se realiza el mapeo de las posiciones de las metas a las dimensiones
        %en  VREP
        Meta2VX=(12.75 * PosMetaX) + 128.5;
        Meta2VY=(12.75 * PosMetaY) + 128.5;
        Meta = zeros(1,2);
        %Se realiza un redondeo de los valores obtenidos al pasa a coordenadas
        %de VREP
        Meta(1) = round(Meta2VX);
        Meta(2) = round(Meta2VY);
        %Se realiza el mapeo de la posicione del Pioneer a las dimensiones
        %en  VREP
        PioneerX= posinicialch(1,1);
        PioneerY= posinicialch(1,2);
        PioneerXMap=(12.750 * PioneerX) + 128.50;
        PioneerYMap=(12.750 * PioneerY) + 128.50;
        PioneerPos = zeros(1,2);
        %Se redondean las posiciones del Pioneer a enteros
        PioneerPos(1) = round(PioneerXMap);
        PioneerPos(2) = round(PioneerYMap);
        %Se realiza la planificacion por medio del algoritmo D*
        ds.plan(Meta);
        recorrido = ds.query(PioneerPos);
        %Se realiza un mapeo del recorrido obtenido a coordenadas de VREP
        recorrido = (recorrido-128.5)/12.75;
        i=1;
        %Ciclo para realizar el recorrido obtenido
        while(k < K)
            tic;  
            PosChasis = zeros(1,3);
            %Se obtiene la posicion del objeto chasis del Pioneer
            [~, PosChasis(1,1:3)]= vrep.simxGetObjectPosition(id, h.robotchasis,-1,vrep.simx_opmode_oneshot_wait);
            posinicialch = zeros(1,2);
            posinicialch(1,1:2)=PosChasis(1,1:2);
            x = posinicialch(1,1);
            y = posinicialch(1,2);
            VPosxy= zeros(1,2);
            VPosxy(1,1)=x;
            VPosxy(1,2)=y;
            %Se obtiene la orientacion del Pioneer
            newPos2 = zeros(1,3);
            [~, newPos2(1,1:3)]= vrep.simxGetObjectOrientation(id, h.robotchasis,-1,vrep.simx_opmode_oneshot_wait);
            ThetaPioneer = newPos2(1,3);
            try
                xg = recorrido(i,1);
                yg = recorrido(i,2);
                vg=zeros(1,2);
                vg(1,1)=xg;
                vg(1,2)=yg;
            catch
                break;
            end
            %Medidas del Pioneer
            r=0.0975;
            l=0.1905;
            %Constantes de Posicion y Orientacion
            Kp = 0.55;
            Ko = 0.90;
            %Se calcula el valor del angulo
            Theta = atan2((yg-y),(xg-x));
            %Se calcula el error de posicion y orientacion
            errorP = norm(vg-VPosxy);            
            errorO = atan2(sin(Theta-ThetaPioneer) , cos(Theta-ThetaPioneer));
            %Si el error es menor al cierto valor el Pioneer deja de moverse
            if (errorP<0.9)
                v=0;
                w=0;
                i = i+1;
            else
                v= Kp*errorP;
                w=Ko*errorO;
            end
            %Se calcula el valor que se ingresara a cada rueda del Pioneer
            Phi_R = (v+w*l)/r;
            Phi_L = (v-w*l)/r;
            %Se ingresan los valores correspondientes a cada rueda para generar
            %el movimiento del mismo
            vrep.simxPauseCommunication(id, true);
            vrep.simxSetJointTargetVelocity(id, h.rightmotor, Phi_R, vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(id, h.leftmotor, Phi_L, vrep.simx_opmode_oneshot);
            vrep.simxPauseCommunication(id, false);       
            k = k + 1;
            %Se verifica que no se esten ejecutando las instrucciones mas rapido
            %que VREP
            elapsed = toc;
            timeleft = timestep - elapsed;
            if timeleft > 0
                pause(min(timeleft, .01));
            end    
        end
        %Se incrementa la bandera de iteraciones
        Iteracion=Iteracion+1;
            %% Se realiza el movimiento final del brazo (llegar del recorrido y dejar el objeto)
        if(Iteracion==1)||(Iteracion==3)||(Iteracion==5)||(Iteracion==7)||(Iteracion==9) 
        IK0 = q';
            %Se selecciona la posicion dependiendo de la iteracion en la que se
            %encuentre
            if Iteracion == 1
            MesaInercialVFinal=[m5xOF,m5yOF,0.6];
            end
            if Iteracion == 3
            MesaInercialVFinal=[m4xOF,m4yOF,0.6];
            end
            if Iteracion == 5
            MesaInercialVFinal=[m3xOF,m3yOF,0.6];
            end
            if Iteracion == 7
            MesaInercialVFinal=[m2xOF,m2yOF,0.6];
            end
            if Iteracion == 9
            MesaInercialVFinal=[m1xOF,m1yOF,0.6];
            end
            %Se establece el offset que se agrega al valor selecionado de la mesa
            offsetMove2=[-3.5,-1.58,0.6];
            MesaInercialVFinal=MesaInercialVFinal+offsetMove2;
            %Se realiza la traspuesta del vector
            PosMetaFinal = MesaInercialVFinal';
            RotPos = rpy2r(double(0),double(0),double(0),'xyz');
            %Se obtiene la pose de la mesa respecto al marco inercial
            InercialTMesaFinal = [RotPos, PosMetaFinal; 0,0,0,1];
            %Se obtiene la pose del UR3 respecto al inercial
            InercialTRobotFinal=[RBrazoUR3, poser'; 0,0,0,1];
            %Se obtiene la traspuesta
            RobotTInercialFinal=inv(InercialTRobotFinal);
            FMesa=RobotTInercialFinal*InercialTMesaFinal
            TMesa=FMesa;
            tMesa=[0.5,0,1]';
            TFinal=[RotPos, tMesa; 0,0,0,1];
            %Se obtiene el vector de un punto intermedio para la trayectoria
            tPoseMedia=[0.3;0;0.6];
            TMedio=[RotPos,tPoseMedia;0,0,0,1];
            %Se obtiene la cinematica inversa para llegar a la mesa
            IK1=ur3IK(TMesa,IK0,1)';
            IK2=ur3IK(TMedio,IK1',1)';
            IK3=ur3IK(TFinal,IK2',1)';
            %Se genera la trayectoria que cumple con las poses que se obtuvieron
            Trj1=mtraj(@lspb, IK0',IK1,200);
            Trj2=mtraj(@lspb, IK1,IK2,200);
            Trj3=mtraj(@lspb, IK2,IK3,200);
            Q=[Trj1;Trj2;Trj3];
            k=1;
            while(k < 601)
                tic; 
                q=Q(k,:);
                if(k==200)
                   open_gripper();
                end
                vrep.simxPauseCommunication(id, true);  
                for i = 1:6
                    vrep.simxSetJointTargetPosition(id, h.armJoints(i), q(i), vrep.simx_opmode_oneshot);
                end
                vrep.simxPauseCommunication(id, false);
                k = k + 1;
                % Se verifica que se encuentre sincronizado con la velocidad de
                % VREP
                elapsed = toc;
                timeleft = timestep - elapsed;
                if timeleft > 0
                    pause(min(timeleft, .01));
                end
            end
        pause(2);
        %Se ingresan los valores correspondientes a cada rueda para parar el
        %Pioneer
        vrep.simxPauseCommunication(id, true);
        vrep.simxSetJointTargetVelocity(id, h.rightmotor, 0, vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(id, h.leftmotor, 0, vrep.simx_opmode_oneshot);
        vrep.simxPauseCommunication(id, false);
        pause(2);
        end
        %Se ingresan los valores correspondientes a cada rueda para parar el
        %Pioneer
        vrep.simxPauseCommunication(id, true);
        vrep.simxSetJointTargetVelocity(id, h.rightmotor, 0, vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(id, h.leftmotor, 0, vrep.simx_opmode_oneshot);
        vrep.simxPauseCommunication(id, false);
        pause(2);
    end
%% Finalizaci?n
% Se termina la comunicaci?n entre MATLAB y V-Rep
vrep.delete(); 
end