clear all
close all
clc

%Medidas de los eslabones del robot
a_1 = .033; 
a_2 = .155;
a_3 = .1348;
d_1 = .2555;
d_5 = .1936;

bot = Bot_youBot();

%Declaracion de todas nuestras matrices de transformacion
T01 = @(theta_1) DH(a_1 ,pi/2  ,d_1 ,theta_1);
T12 = @(theta_2) DH(a_2 ,0     ,0   ,theta_2+pi/2);
T23 = @(theta_3) DH(a_3 ,0     ,0   ,theta_3);
T34 = @(theta_4) DH(0   ,-pi/2 ,0   ,theta_4-pi/2);
T45 = @(theta_5) DH(0   ,0     ,d_5 ,theta_5);

%Dibujar_Manipulador({T01(q(1)) T12(q(2)) T23(q(3)) T34(q(4)) T45(q(5))},{'RRRRR'},0.2);

%Intervalos de tiempo y duracion de la simulacion
dt = 0.05;
S = 20;

%Declaracion de variables para graficar
q_plot = [];
qp_plot = [];
xi_plot = [];
xd_plot = [];
t_plot = [];

%Ganancia del controlador
K = diag([1 1 1]);

%Calcular Matriz de transformacion actual
Td = bot.ReferenceFrame_Pose();
q = bot.Get_Joint_Position();
T = T01(q(1))*T12(q(2))*T23(q(3))*T34(q(4))*T45(q(5));

%Posicion inicial y final de la planificacion de trayectorias
x1 = T(1:3,4);
x2 = Td(1:3,4);
a = movimiento(x1,x2,S);


for t=dt:dt:S
    %Obtencion de Posicion deseada y valores articulares actuales
    Td = bot.ReferenceFrame_Pose();
    q = bot.Get_Joint_Position();
    
    %Obtencion del Jacobiano, la Matriz de transformacion y error actuales
    J = Jacobiano(T01(q(1)),T12(q(2)),T23(q(3)),T34(q(4)),T45(q(5)));
    Jv = J(1:3,:);
    T = T01(q(1))*T12(q(2))*T23(q(3))*T34(q(4))*T45(q(5));
    v = Td(1:3,4)-T(1:3,4);
    
    %Calcular la velocidad para nuestras articulaciones
    xd = a*[1 t t^2 t^3]';
    xdp = a*[0 1 2*t 3*t^2]';
    xp = xdp + K*(xd - T(1:3,4));

    %Seguimiento de posicion
    %qp = pinv(Jv)*K*v;

    %Planificacion de trayectorias
    qp = pinv(Jv)*xp;

    %Avanzar en un instante de tiempo determinado la velocidad obtenida
    bot.Set_Joint_Velocity(qp);
    bot.Simulation_Step();

    %Almacenar datos
    q_plot = [q_plot q];
    qp_plot = [qp_plot qp];
    xi_plot = [xi_plot T(1:3,4)];
    xd_plot = [xd_plot x2];
    t_plot = [t_plot t];
end

bot.Stop_Simulation();

%   Posición articular VS tiempo.
figure
hold on
grid on
plot(t_plot, q_plot');
legend('\theta 1','\theta 2','\theta 3','\theta 4','\theta 5')
xlabel('Segundos');
ylabel('rad');

%   Velocidad articular VS tiempo.
figure
hold on
grid on
plot(t_plot, qp_plot');
legend('\theta 1','\theta 2','\theta 3','\theta 4','\theta 5')
xlabel('Segundos');
ylabel('rad/seg');

%   Posición deseada y posición actual del end effector. (x, y, z)
figure
hold on
grid on
plot(t_plot,xd_plot,'LineWidth',3);
legend('x','y','z')
plot(t_plot,xi_plot,'--','LineWidth',3);
legend('x','y','z')
xlabel('Posicion deseada');
ylabel('Posicion actual');


%% Funciones
function T = DH (a,alpha,d,theta)
T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta); ...
sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta); ...
0 sin(alpha) cos(alpha) d; 0 0 0 1];
end

function a = movimiento(qi,qf,tf)
        a =  [qi zeros(3,1) qf zeros(3,1)]*[1 0 0 0;
             0 1 0 0;
             -(3/tf^2) -(2/tf) (3/tf^2) -(1/tf);
             (2/tf^3) (1/tf^2) -(2/tf^3) (1/tf^2)]';
end

function J = Jacobiano(T01,T12,T23,T34,T45)
    T02 = T01*T12;
    T03 = T01*T12*T23;
    T04 = T01*T12*T23*T34;
    T05 = T01*T12*T23*T34*T45;
    
    %Dibujar_Manipulador({T01f(q(1)) T12(q(2)) T23(q(3)) T34(q(4))},{'RRPR'},0.05);
    
    Z0 = [0 0 1]';
    Z1 = T01(1:3,3);
    Z2 = T02(1:3,3);
    Z3 = T03(1:3,3);
    Z4 = T04(1:3,3);
    
    t0 = [0 0 0]';
    t1 = T01(1:3,4);
    t2 = T02(1:3,4);
    t3 = T03(1:3,4);
    t4 = T04(1:3,4);
    t5 = T05(1:3,4);
    
    Jv = [cross(Z0,t5-t0) cross(Z1,t5-t1) cross(Z2,t5-t2) cross(Z3,t5-t3) cross(Z4,t5-t4)];
    Jw = [Z0 Z1 Z2 Z3 Z4];
    J = [Jv;Jw];
end