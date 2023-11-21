%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autores: Equipo Knight Flight Technologies
% Fecha de creación: Semana 4  
% Fecha de última modificación: Semana 5
% Descripción: Código para simular el vuelo de dos drones siguiendo al
% líder virtual
% Formato de nombrar variables y funciones: Minúsculas y descriptivas
% Comentado por: Carlos Flores
%Nota: Para un dron se elimina uno de los lados (con la finalidad de
%simplificar la lógica de vuelo)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all %Clear all variables from the workspace
close all %Close all open figure windows
clc %Clear the command window


%Position of the virtual leader
x = 0;
y = 0;

%Position of Drone 1
x_drone1 = -1;
y_drone1 = -1;

%Position of Drone 2
x_drone2 = 0;
y_drone2 = -1;


kp_drone = 5; %Control constants for drones

l = 0.2; %Distance between drones

%Times and related
t = 0;
Tf = 0.1;
dt = 0.01;


theta =  0.7854; %Initial angle 


kt = 10;
kr = 100;


xd = (0:0.1:4); % Vector of desired points in x
yd = (sin(xd));% Vector of desired points in and using sine function
[filas, columnas] = size(xd); 

figure(1);
plot(xd,yd); %Graph of the sine function to follow
hold on

dc = 0.5; %Known distances for the geometric reference

for i = 1:columnas %For loop to traverse each desired point


   xdd = xd(i);
   ydd = yd(i);

   p1x = x - dc*sin(theta);
   p1y = y + dc*cos(theta);

   p2x = x + dc*sin(theta);
   p2y = y - dc*cos(theta);

   figure(1)
   scatter(p1x,p1y)
   scatter(p2x, p2y)

   t=0;

while (t<Tf) %While loop to reach each desired point in the sine function

    xe = x - xdd;
    ye = y - ydd;

    thetad = atan2(ydd-y,xdd-x);
    thetae = theta - thetad;

    v = kt*sqrt(xe^2 + ye^2);
    omega = -kr*thetae;

    if v > 1
        v = 1;
    end
    if omega > pi/2
        omega = pi/2;
    end
    if omega < -pi/2
        omega = -pi/2;
    end

    vr = v + 0.5*l*omega;
    vl = v - 0.5*l*omega;

    v = (vr + vl)/2;
    omega = (vr - vl)/l;

    xp = v*cos(theta);
    yp = v*sin(theta);
    thetap = omega;

    %Subtract to get drone 1 speeds
    vx_drone1 = (p1x - x_drone1)*kp_drone;
    vy_drone1 = (p1y - y_drone1)*kp_drone;

    %Subtract to get drone 2 speeds 
    vx_drone2 = (p2x - x_drone2)*kp_drone;
    vy_drone2 = (p2y - y_drone2)*kp_drone;

    %Integrate to obtain leader position
    x = x + xp*dt;
    y = y + yp*dt ;
    
    %Integrate to obtain position of drone 1
    x_drone1 = x_drone1 + vx_drone1*dt;
    y_drone1 = y_drone1 + vy_drone1*dt;

    %Integrate to obtain position of drone 2
    x_drone2 = x_drone2 + vx_drone2*dt;
    y_drone2 = y_drone2 + vy_drone2*dt;
    

    theta = theta + thetap*dt;

    t = t + dt;

    %error_x = abs((xdd-x))/abs(xdd);
    %error_y = abs((ydd-y))/abs(ydd);


    figure(1)
    scatter(x,y)
    scatter(x_drone1, y_drone1)
    scatter(x_drone2, y_drone2)
    
end
end
