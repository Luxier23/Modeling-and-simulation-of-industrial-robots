clear all
close all
clc

%% Funcion para calcular la cinematica inversa
function qf = Cinematica_Inversa_Scara(q,posicion)
    if posicion == 1
        theta2 = acos((q(1,4)^2 + q(2,4)^2 - 0.467^2 - 0.4005^2)/(2*0.467*0.4005));
    else
        theta2 = -acos((q(1,4)^2 + q(2,4)^2 - 0.467^2 - 0.4005^2)/(2*0.467*0.4005));
    end
    theta1 = atan2(q(2,4),q(1,4)) - asin((0.4005*sin(theta2))/sqrt(q(1,4)^2 + q(2,4)^2));
    d3 = 0.4 - q(3,4) - 0.127;
    theta4 = theta1 + theta2 - atan2(q(2,1),q(1,1));

    qf = [theta1 theta2 d3 theta4]';
end

function a = movimiento(qi,qf,tf)
        a =  [qi zeros(4,1) qf zeros(4,1)]*[1 0 0 0;
             0 1 0 0;
             -(3/tf^2) -(2/tf) (3/tf^2) -(1/tf);
             (2/tf^3) (1/tf^2) -(2/tf^3) (1/tf^2)]';
end

%% Inicio del código
S = 10 %Tiempo de ejecucion

bot = Bot_Scara();
Posicion_Rectangulo = bot.Rectangle_Pose();
Posicion_Final = bot.ReferenceFrame_Pose();
Distancia_Rectangulo = sqrt(Posicion_Rectangulo(1,4)^2 + Posicion_Rectangulo(2,4)^2);
Distancia_Final = sqrt(Posicion_Final(1,4)^2 + Posicion_Final(2,4)^2);

%% Validar posiciones
bandera = 1;
if ((Distancia_Rectangulo > 0.467+0.4005)|(Posicion_Rectangulo(3,4)>(0.4-0.127))|(Posicion_Rectangulo(3,4)<0))
    disp("Rectangulo fuera de alcance")
    bot.Stop_Simulation()
    bandera = 0;
end

if ((Distancia_Final > 0.467+0.4005)|((Posicion_Final(3,4)+Posicion_Rectangulo(3,4))>(0.4-0.127))|(Posicion_Final(3,4)<0))
    disp("Objetivo fuera de alcance")
    bot.Stop_Simulation()
    bandera = 0;
end

if bandera == 1
    disp("Es posible ejecutar el código")
    
    %% Valores iniciales y primer objetivo
    
    qi = [0 0 0 0]';
    qf = Cinematica_Inversa_Scara(Posicion_Rectangulo,1);
    altura_objetivo = qf(3);
    qf(3) = 0;
    %% Primer Movimiento, acercarse al objeto
    s = S/5;
    a = movimiento(qi,qf,s);
        tic
        while toc <= s
            q1 = a*[1 toc toc^2 toc^3]';
            bot.Set_Joint_Position(q1);
        end
    %% Segundo movimiento, recoger objeto
    qi = bot.Get_Joint_Position();
    qf(3) = altura_objetivo;
    s = S/10 - .1;
    a = movimiento(qi,qf,s);
        tic
        while toc <= s
            q1 = a*[1 toc toc^2 toc^3]';
            bot.Set_Joint_Position(q1);
        end
    bot.Gripper_Command(0);
    pause(.1);

    %% Tercer movimiento, subir objeto
    qi = bot.Get_Joint_Position();
    qf(3) = 0;
    s = S/10;
    a = movimiento(qi,qf,s);
        tic
        while toc <= s
            q1 = a*[1 toc toc^2 toc^3]';
            bot.Set_Joint_Position(q1);
        end
    
    %% Cuarto movimiento, acercarse a posicion final
    qi = bot.Get_Joint_Position();
    qf = Cinematica_Inversa_Scara(Posicion_Final,1);
    altura_final = qf(3) - (.4-.127-altura_objetivo+.0205);
    qf(3) = 0;
    s = S/5;
    a = movimiento(qi,qf,s);
        tic
        while toc <= s
            q1 = a*[1 toc toc^2 toc^3]';
            bot.Set_Joint_Position(q1);
        end
    
    %% Quinto movimiento, dejar objeto
    qi = bot.Get_Joint_Position();
    qf(3) = altura_final;
    s = S/10 -.1;
    a = movimiento(qi,qf,s);
        tic
        while toc <= s
            q1 = a*[1 toc toc^2 toc^3]';
            bot.Set_Joint_Position(q1);
        end
    bot.Gripper_Command(1);
    pause(.1);
    
    %% Sexto movimiento, subir actuador
    qi = bot.Get_Joint_Position();
    qf(3) = 0;
    s = S/10;
    a = movimiento(qi,qf,s);
        tic
        while toc <= s
            q1 = a*[1 toc toc^2 toc^3]';
            bot.Set_Joint_Position(q1);
        end
    %% Sexto movimiento, regresar a su posicion inicial
    qi = bot.Get_Joint_Position();
    qf = [0 0 0 0]';
    s = S/5;
    a = movimiento(qi,qf,s);
        tic
        while toc <= s
            q1 = a*[1 toc toc^2 toc^3]';
            bot.Set_Joint_Position(q1);
        end
    pause(3)

end

bot.Stop_Simulation();