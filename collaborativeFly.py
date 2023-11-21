##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 8  
# Fecha de última modificación: Semana 9
# Descripción: Código para mediante un lider virtual (con ruta senoidal) se genere un vuelo coordinado entre dos drones
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Cristóbal Padilla
#

##################################################################################################################################################


'''
Librerias
'''
import math
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
import matplotlib.pyplot as plt
import numpy as np

'''
Variables globales
'''
# URI of the drones to which they are going to connect
uri1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')
uri2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')


'''
Funciones
'''
#Kalman Filter
def wait_for_position_estimator(scf):    
     '''
    Hace el calculo de la estimacion del dron una vez se reinicia
        Parameters:
            scf (): 
    '''
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            if (max_x - min_x) < threshold and (max_y - min_y) < threshold and (max_z - min_z) < threshold:
                break
#Function that recives the initial positions of the dron and uses the kalman filter
def set_initial_position(scf, x, y, z, yaw_deg):
     '''
    Establece la posición inicial del dron y hace las estimaciones correctas

            Parameters:
                    scf (): 
                    x (float): Posición inicial en el eje x del dron
                    y (float): Posición inicial en el eje y del dron
                    z (float): Posición inicial en el eje z del dron
                    yaw_deg (int): Another decimal integer
    '''
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


#Function to reset the drone estimator
def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

#Function that calls the sequence of the points that the drones hav to follow
def run_sequence(scf, base_x, base_y, base_z, yaw, base_x2, base_y2, base_z2, scf2):

    #Drone and drone 2 instances are established
    cf = scf.cf
    cf2 = scf2.cf

    # The axes for the graph are established
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('center')
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')

    # The vectors for the sinusoidal function to follow are made and graphed in blue
    x1 = np.linspace(0, 1.5*np.pi, 25) + 0.5 
    y1 = np.sin(x1)+1.5
    line1, = ax.plot(y1, x1, 'b')

    # THE BOUNDARIES OF THE WORK AREA ARE GRAPHIC IN GREEN########################################
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
    #################################################################################################

    # The limits of the X and Y axes are established
    ax.set_xlim(-3, 2*np.pi)
    ax.set_ylim(-3, 7)

    # Empty vectors are generated where the movement of the virtual leader in red will be graphed in real time.
    x2 = []
    y2 = []
    line2, = ax.plot([], [], 'r')


    #------------------------------------------------------------------------

    #Start variables for the virtual leader

    #Virtual leader position
    x_lider = 0.5
    y_lider = 2.0

    #Car variables and time
    l = 0.2
    t = 0
    Tf = 0.1
    dt = 0.01

    #Initial angle of the leader and control constants
    theta =  0.7854
    kt = 10
    kr = 100

    #Known distances for geometric reference
    dc = 0.25


    #------------------------------------------------------------------------

    #They are the desired points to follow that the virtual leader generates.
    punto_deseado1, = ax.plot(0, 0, marker='*', color='red')
    punto_deseado2, = ax.plot(0, 0, marker='*', color='red')

    #Take off drones where they start and keep them at 1 meter high
    cf.commander.send_position_setpoint(base_x, base_y, 1.0, yaw)
    cf2.commander.send_position_setpoint(base_x2, base_y2, 1.0, yaw)
    time.sleep(0.1)


    # Start the initial loop with the movement of the leader 
    for i in range(len(x1)):

        #Desired point of the virtual leader
        xd = x1[i]
        yd = y1[i]
        
        #Desired points in X and Y for the first desired point
        p1x = x_lider - dc*np.sin(theta)
        p1y = y_lider + dc*np.cos(theta)

        #Desired points in X and Y for the second desired point
        p2x = x_lider + dc*np.sin(theta)
        p2y = y_lider - dc*np.cos(theta)

        #Current error of the leader's desired point and current position
        xe = x_lider - xd
        ye = y_lider - yd 

        #Cycle that is executed until there is an error of less than 5% in the leader in X and Y
        while (abs(xe)>0.05 or abs(ye)>0.05 ):

            #Error calculation
            xe = x_lider - xd
            ye = y_lider - yd 

            #The desired angle and the error of the virtual leader are calculated.
            theta_d = math.atan2(yd - y_lider, xd - x_lider)
            theta_e = theta - theta_d

            #The speed of the leader is calculated as well as the angular velocity
            v = kt*math.sqrt(xe**2 + ye**2)
            omega = -kr*theta_e

            #Speed ​​saturation
            if(v>1):
                v = 1
            if(omega>(np.pi/2)):
                omega = np.pi/2        
            if(omega<(-np.pi/2)):
                omega = -np.pi/2   

            #Calculation of speeds per tire
            vr = v + 0.5*l*omega
            vl = v - 0.5*l*omega

            #Calculation of complete linear and angular velocities 
            v = (vr + vl)/2
            omega = (vr - vl)/l

            #The derivative of the position in X, Y and theta is calculated.
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

            # The position of the leader is added to the empty lists
            x2.append(x_lider)
            y2.append(y_lider)
            
            # The position of the leader in red is updated in the graph.
            line2.set_xdata(y2)
            line2.set_ydata(x2)

            #Update new data in graph for the desired points calculated
            punto_deseado1.set_data(p1y, p1x)
            punto_deseado2.set_data(p2y, p2x)
            
            # The complete graph is drawn with the route to follow, the leader and desired points.
            plt.draw()
            
            # Pause to graph the data well
            plt.pause(0.01) 
            #---------------------------------------------------------
            #The positions to be sent for each drone are calculated depending on where it starts and the points.
            #desired calculated by the leader
            x_drone1 = p1y + base_x
            y_drone1 = p1x + base_y
            z_drone1 = 1.0

            x_drone2 = p2y + base_x2
            y_drone2 = p2x + base_y2
            z_drone2 = 1.0
        
        #The calculated points are sent to each drone
        cf.commander.send_position_setpoint(x_drone2, y_drone2, z_drone2, yaw)
        cf2.commander.send_position_setpoint(x_drone1, y_drone1, z_drone1, yaw)
        time.sleep(0.1)

    # The final graph is shown
        plt.show()

    #Drones are sent to a low altitude to land
    cf.commander.send_position_setpoint(x_drone1, y_drone1, 0.15, yaw)
    cf2.commander.send_position_setpoint(x_drone2, y_drone2, 0.15, yaw)

    time.sleep(2)

    #The flight ends and the engines stop
    cf.commander.send_stop_setpoint()
    cf2.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Position of drone 1 in X,Y,Z and orientation in yaw
    initial_x = 2.70
    initial_y = 3.15
    initial_z = 0.0
    initial_yaw = 90  # In degrees
    
    #Position of drone 2 in X,Y,Z and orientation in yaw
    initial_x2 = 0.2
    initial_y2 = 3.15
    initial_z2 = 0.0



    #Drone 1 connection  
    with SyncCrazyflie(uri1, cf=Crazyflie(rw_cache='./cache')) as scf:

        #Set initial position of drone 1
        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        #reiniciar estimador para el dron 1 
        reset_estimator(scf)
        
        #Drone 2 connection 
        with SyncCrazyflie(uri2, cf=Crazyflie(rw_cache='./cache')) as scf2:

            #Set initial position of drone 2
            set_initial_position(scf2, initial_x2, initial_y2, initial_z2, initial_yaw)
            #Reset estimator for drone 2
            reset_estimator(scf2)

            #Start the sequence where the drones follow the virtual leader
            run_sequence(scf, initial_x, initial_y, initial_z, initial_yaw, initial_x2, initial_y2, initial_z2, scf2)
