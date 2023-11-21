##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 4  
# Fecha de última modificación: Semana 6
# Descripción: Código para simular en python la ruta senoidal junto con los drones siguiendo los puntos deseados
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Carlos Flores

##################################################################################################################################################
#Import libraries
import matplotlib.pyplot as plt
import numpy as np
import math
import threading
import time

#We initialize the drone positions as desired (they will be updated)
puntoDeseado_x_dron1 = 0
puntoDeseado_y_dron1 = 0

puntoDeseado_x_dron2 = 0
puntoDeseado_y_dron2 = 0

flagEndDataThread = True

def liderVirtual():

    # Set up the plot with the axes at the centre
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('center')
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')

    # Set up the initial plot for the function from -5 to 5 in blue
    x1 = np.linspace(0, 1.0*np.pi, 50) + 0.5 #el numero final de la funcion linspace es la cantidad de puntos, inico en 25
    y1 = 0.5*np.sin(x1)+1.5
    line1, = ax.plot(y1, x1, 'b')

    leftLine_y = np.linspace(0, 6.3, 2)
    leftLine_x = 0.0 + leftLine_y*0
    leftLine, = ax.plot(leftLine_x, leftLine_y, 'g')

    rightLine_y = np.linspace(0, 6.3, 2)
    rightLine_x = 2.95 + rightLine_y*0
    rightLine, = ax.plot(rightLine_x, rightLine_y, 'g')

    topLine_y = np.linspace(0.0, 2.95, 2)
    topLine_x = 6.3 + topLine_y*0
    topLine, = ax.plot(topLine_y, topLine_x, 'g')

    botLine_y = np.linspace(0.0, 2.95, 2)
    botLine_x = 0 + botLine_y*0
    botLine, = ax.plot(botLine_y, botLine_x, 'g')


    # Set the x and y limits of the plot
    ax.set_xlim(-3, 2*np.pi)
    ax.set_ylim(-3, 7)

    # Create empty lists to store x and y values for the updated plot in red
    x2 = []
    y2 = []
    line2, = ax.plot([], [], 'r')

    #------------------------------------------------------------------------

    #Start variables for the virtual leader

    #Position of the virtual leader
    x_lider = 0.5
    y_lider = 1.72

    #Car kinematics variables transformed to drones and time
    l = 0.2
    t = 0
    Tf = 0.1
    dt = 0.01

    #Initial angle of the leader and control constants
    theta =  0.7854
    kt = 10 #estaba en 10
    kr = 100

    #Known distances for the geometric reference
    dc = 0.5


    #------------------------------------------------------------------------

    punto_deseado1, = ax.plot(0, 0, marker='*', color='red')
    punto_deseado2, = ax.plot(0, 0, marker='*', color='red')

    #Set global variables
    global puntoDeseado_x_dron1
    global puntoDeseado_y_dron1

    global puntoDeseado_x_dron2
    global puntoDeseado_y_dron2

    global flagEndDataThread

    # We start the loop to update the graph
    for i in range(len(x1)):

        xd = x1[i]
        yd = y1[i]

        t = 0

        
        p1x = x_lider - dc*np.sin(theta)
        p1y = y_lider + dc*np.cos(theta)

        
        p2x = x_lider + dc*np.sin(theta)
        p2y = y_lider - dc*np.cos(theta)


        puntoDeseado_x_dron1 = p1x
        puntoDeseado_y_dron1 = p1y
        puntoDeseado_x_dron2 = p2x
        puntoDeseado_y_dron2 = p2y
        #Obtenemos el error
        xe = x_lider - xd
        ye = y_lider - yd 

        
        #Conditional to work while within treshold
        while (abs(xe)>0.05 or abs(ye)>0.05 ):

            xe = x_lider - xd
            ye = y_lider - yd 

            theta_d = math.atan2(yd - y_lider, xd - x_lider)
            theta_e = theta - theta_d

            v = kt*math.sqrt(xe**2 + ye**2)
            omega = -kr*theta_e

            #Speed ​​saturation to avoid extravagant inputs
            if(v>1):
                v = 1
            if(omega>(np.pi/2)):
                omega = np.pi/2        
            if(omega<(-np.pi/2)):
                omega = -np.pi/2   

            vr = v + 0.5*l*omega
            vl = v - 0.5*l*omega

            v = (vr + vl)/2
            omega = (vr - vl)/l

            #Speeds 
            xp = v*math.cos(theta)
            yp = v*math.sin(theta)
            thetap = omega

            #Integrate to obtain leader position
            x_lider = x_lider + xp*dt
            y_lider = y_lider + yp*dt 
            theta = theta + thetap*dt

            t = t + dt
            #---------------------------------------------------------
            #GRAPH, BLOCK

            # Append the new x and y values
            x2.append(x_lider)
            y2.append(y_lider)
            
            # Update the plot with the new x and y values in red
            line2.set_xdata(y2)
            line2.set_ydata(x2)

            #Update new data in graph REMEMBER THEY ARE PLACED REVERSE BECAUSE IN REAL LIFE IT IS REVERSE DOWN
            
            punto_deseado1.set_xdata([p1y])
            punto_deseado1.set_ydata([p1x])

            punto_deseado2.set_xdata([p2y])
            punto_deseado2.set_ydata([p2x])
            
            # Draw the updated plot
            plt.draw()
            
            # Pause to allow the plot to update
            plt.pause(0.01) #vemos dudoso si dejarlo...
            #---------------------------------------------------------
            #GRAPH, BLOCK


####Function to print the desired coordinates
def imprimiendoDatosDeseados():
    global puntoDeseado_x_dron1
    global puntoDeseado_y_dron1
    global puntoDeseado_x_dron2
    global puntoDeseado_y_dron2

    global flagEndDataThread

    while flagEndDataThread:
        print("Dron 1: ", puntoDeseado_x_dron1, ", ", puntoDeseado_y_dron1)
        print("Dron 2: ", puntoDeseado_x_dron2, ", ", puntoDeseado_y_dron2)
        time.sleep(0.1)
    

    def threading_init():
        # Initialize threading for concurrent execution
    
        # Uncomment the following line if there is another thread for liderVirtual
        # hilo_liderVirtual = threading.Thread(target=liderVirtual)
    
        # Create a thread for liderVirtual
        hilo_data = threading.Thread(target=liderVirtual)
    
        # Uncomment the following lines if there are multiple threads
        # Start the thread for liderVirtual
        # hilo_liderVirtual.start()
    
        # Start the thread for liderVirtual
        hilo_data.start()
    
        # Uncomment the following lines if there are multiple threads
        # Wait for the thread for liderVirtual to complete
        # hilo_liderVirtual.join()
    
        # Wait for the thread for liderVirtual to complete
        hilo_data.join()

    

#threading_init()

if __name__ == '__main__':

    liderVirtual()
