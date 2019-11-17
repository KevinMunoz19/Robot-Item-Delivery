function q = ur3IK(Td, q0, solop)
    tol_p = 1e-06; % Tolerancia del error de posición
    tol_o = 1e-05; % Tolerancia del error de orientación
    %Se define el numero de Iteraciones
    K = 50; 
    % Inicialización de variables
    q = q0;
    k = 0;
    lambda=0.1;
    TI=ur3FK(q);
    Rd=Td(1:3,1:3);  %Matriz de rotacion
    Rk=TI(1:3,1:3);
    e_p=Td(1:3,4)-TI(1:3,4);
    %Error de orientacion
    Ed= rot2cuat(Rd);
    Ek=rot2cuat(Rk);
    Ei= invcuat(Ek);
    DeltaE= multcuat(Ed,Ei);
    e_o=DeltaE(2:4,1);
    while((norm(e_p) > tol_p) && (norm(e_o) > tol_o) && (k < K))
        J=ur3J(q);
        TI=ur3FK(q);
        e_p=Td(1:3,4)-TI(1:3,4);
        Jv= J(1:3,:);
        %METODO DE LEVENVERG-MARQUARDT
        if(solop==1)
            I=eye(3);
            Ji = Jv'*inv(Jv*Jv'+(lambda)*(lambda)*I);
            q = q + Ji*e_p; 
            k = k + 1;
        elseif(solop==0)
            I=eye(6);
            Ji_T = J'*inv(J*J'+(lambda)*(lambda)*I);
            TI=ur3FK(q);
            Rk=TI(1:3,1:3);
            Ed= rot2cuat(Rd);
            Ek=rot2cuat(Rk);
            Ei= invcuat(Ek);
            DeltaE= multcuat(Ed,Ei);
            e_o=DeltaE(2:4,1);
            e = [e_p; e_o]; 
            q = q + Ji_T*e; 
            k = k + 1;
        end
    end