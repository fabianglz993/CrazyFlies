##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 9  
# Fecha de última modificación: Semana 9
# Descripción: Código para mediante un lider virtual (con ruta senoidal) se genere un vuelo coordinado entre dos drones con hilos y campos potenciales
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Correciones de comentarios por: Natalia Rodríguez González

# Comentado por: Cristóbal Padilla

##################################################################################################################################################
#Import standar libraries
import math
import time
import keyboard as kb
import threading
import pandas
import matplotlib.pyplot as plt
import numpy as np

#Import libraries to communicate with Crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper



#The desired points are established where the drones should arrive initially, in X and Y are where you put the drone on the map at the beginning
# and in Z is the height you want it to reach at the beginning, to stabilize first.

#Position of Done 1¿
puntoDeseado_x_dron1 = 1.0
puntoDeseado_y_dron1 = 0.2
puntoDeseado_z_dron1 = 1.0

#Position of Done 2
puntoDeseado_x_dron2 = 2.0
puntoDeseado_y_dron2 = 0.2
puntoDeseado_z_dron2 = 1.0

#Flag to determine if the virtual leader is still running or has already finished executing
liderAunCorreFlag = True

#Global variables to know the position of drone 1 with the LOGS
X1 = 0
Y1 = 0
Z1 = 0

VX1 = 0
VY1 = 0
VZ1 = 0

#Global variables to know the position of drone 2 with the LOGS
X2 = 0
Y2 = 0
Z2 = 0

VX2 = 0
VY2 = 0
VZ2 = 0


# URI of the crazyfly to which it will connect. What corresponds to drone 1 corresponds to the normal URI, which
# of drone2 corresponds to uri2.
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703')
uri2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')


#Arrays where the recolected data of both drones will be stored while the code is running.
FINAL_ARRAY = []
FINAL_ARRAY2 = []


#Function that calls the Virtual leader and its graphs with the desired points and drone positions in real time
#NOTE: This function has to be run in the main code thread, since the matplotlib libraries require it.
def liderVirtual():
    # Set the axis of the graph.  
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('center')
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')

    # The vector is generated with the sine function that the virtual leader must follow and is graphed in blue.
    x1 = np.linspace(0, 1.5*np.pi, 50) + 0.5 
    y1 = 0.5*np.sin(x1)+1.5
    line1, = ax.plot(y1, x1, 'b')

    ########### The square is generated (work area) of the drones in green################################
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

    ################ End of the work area generation################################ 


    # The limits of the graph are defined (for display only)
    ax.set_xlim(-3, 2*np.pi)
    ax.set_ylim(-3, 7)

    # Empty lists are generated that will store the route followed by the virtual leader in red
    x2 = []
    y2 = []
    line2, = ax.plot([], [], 'r')


    #------------------------------------------------------------------------

    #Start variables for the virtual leader

    #Initial position of th evirtual leader
    x_lider = 0.5
    y_lider = 1.72

    #Car variables and time
    l = 0.2
    t = 0
    Tf = 0.1
    dt = 0.01

    #Initial angle of the leader and control constants
    theta =  0.7854
    kt = 5 
    kr = 100

    #Known distances for the geometric reference, from the center to the desired point
    dc = 0.5


    #------------------------------------------------------------------------
    #Instances are created to plot the desired points in real time in red.
    punto_deseado1, = ax.plot(0, 0, marker='*', color='red')
    punto_deseado2, = ax.plot(0, 0, marker='*', color='red')

    #Instances are created to graph the position of the drones in real time in blue.
    dron1_posActual, = ax.plot(0, 0, marker='*', color='blue')
    dron2_posActual, = ax.plot(0, 0, marker='*', color='blue')

    #The global variables of the desired points are taken for each drone
    global puntoDeseado_x_dron1
    global puntoDeseado_y_dron1
    global puntoDeseado_x_dron2
    global puntoDeseado_y_dron2
    
    #The position of the drones is taken from the global variables that are updated in the logs
    global X1
    global Y1
    global X2
    global Y2

    #Time for the drones to fly calmly, stabilize and then the virtual leader begins
    time.sleep(5)

    # Inicia la secuencia para el líder virtual, se generan los puntos deseados y los drones se mueven en formación.
    for i in range(len(x1)):

        #The next desired point is taken for the virtual leader
        xd = x1[i]
        yd = y1[i]
        t = 0 #Time is reset to 0 every time you move to a desired point

        #The desired points are calculated in geometric formation for drone 2
        p2x = x_lider - dc*np.sin(theta)
        p2y = y_lider + dc*np.cos(theta)

        #The desired points are calculated in geometric formation for drone 1
        p1x = x_lider + dc*np.sin(theta)
        p1y = y_lider - dc*np.cos(theta)

        #The global variables of desired points are updated with the calculations of the virtual leader
        puntoDeseado_x_dron1 = p1y
        puntoDeseado_y_dron1 = p1x
        puntoDeseado_x_dron2 = p2y
        puntoDeseado_y_dron2 = p2x

        #Virtual leader position error
        xe = x_lider - xd
        ye = y_lider - yd 
        
        #While loop to define when the virtual leader reaches a desired point
        while (abs(xe)>0.05 or abs(ye)>0.05 ):

            #Error
            xe = x_lider - xd
            ye = y_lider - yd 

            #Angle and error
            theta_d = math.atan2(yd - y_lider, xd - x_lider)
            theta_e = theta - theta_d

            #The speed of the leader is calculated as well as its angular velocity
            v = kt*math.sqrt(xe**2 + ye**2)
            omega = -kr*theta_e

            #Both speeds are saturated so that it does not go too fast and does not turn too abruptly.
            if(v>1):
                v = 1
            if(omega>(np.pi/2)):
                omega = np.pi/2        
            if(omega<(-np.pi/2)):
                omega = -np.pi/2   

            #Calculation of speeds on each tire for the virtual leader
            vr = v + 0.5*l*omega
            vl = v - 0.5*l*omega

            #Spped calculation
            v = (vr + vl)/2
            omega = (vr - vl)/l

            #Calculation of the derivatives of position and angle
            xp = v*math.cos(theta)
            yp = v*math.sin(theta)
            thetap = omega

            #Integrate to obtain leader position
            x_lider = x_lider + xp*dt
            y_lider = y_lider + yp*dt 
            theta = theta + thetap*dt

            #Simulation time accumulates
            t = t + dt

            #---------------------------------------------------------
            #############################PART TO GRAPH DATA IN REAL TIME###############################

            # The position data of the virtual leader is added to be graphed
            x2.append(x_lider)
            y2.append(y_lider)
            
            # The graph of the virtual leader is updated with his position and path traveled in red
            line2.set_xdata(y2)
            line2.set_ydata(x2)

            #The data of the desired points and the drones are updated in real time
            
            punto_deseado1.set_xdata([p1y])
            punto_deseado1.set_ydata([p1x])

            punto_deseado2.set_xdata([p2y])
            punto_deseado2.set_ydata([p2x])

            dron1_posActual.set_xdata([X1])
            dron1_posActual.set_ydata([Y1])

            dron2_posActual.set_xdata([X2])
            dron2_posActual.set_ydata([Y2])
            
            # The graph is drawn with all the data
            plt.draw()
            
            # Pause to update data correctly
            plt.pause(0.01) 
            #---------------------------------------------------------
            ####################################END OF GRAPHING BLOCK###################################################
    
    #####The desired points for z of both drones are taken and lowered to 0 for 5 seconds. This is done so that they are in a safe position before landing.###############
    global puntoDeseado_z_dron1
    global puntoDeseado_z_dron2

    puntoDeseado_z_dron1 = 0.0
    puntoDeseado_z_dron2 = 0.0

    #Print to show that both drones are landing 
    print("Aterrizando . . .")
    time.sleep(5)

    #The flag is taken if the leader is still running to set it to false.
    #This causes both threads (drone 1 and drone 2) to finish their execution and therefore the code ends
    global liderAunCorreFlag
    liderAunCorreFlag = False

############CALLBACKS SECTION FOR DRONES##################################
###Postition Callback####
def pos_angCallback(timestamp, data, logconf):
    
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    global X1
    global Y1
    global Z1



    X1 = data['stateEstimate.x']
    Y1 = data['stateEstimate.y']
    Z1 = data['stateEstimate.z']

    FINAL_ARRAY.append(dic_temp)
   
###Speed Callback####
def velCallback(timestamp, data, logconf):

    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    global VX1
    global VY1
    global VZ1

    VX1 = data['stateEstimate.vx']
    VY1 = data['stateEstimate.vy']
    VZ1 = data['stateEstimate.vz']

    FINAL_ARRAY.append(dic_temp)

###Quaternion Callback####
def quaternionCallback(timestamp, data, logconf):

    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)

##Angles Rate Callback####
def anglesRatesCallback(timestamp, data, logconf):

    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)

#############Drone 2 callbacks###################
###Postition Callback####
def pos_angCallback2(timestamp, data, logconf):

    global FINAL_ARRAY2
    dic_temp = data
    dic_temp.update({"time": timestamp})

    global X2
    global Y2
    global Z2


    X2 = data['stateEstimate.x']
    Y2 = data['stateEstimate.y']
    Z2 = data['stateEstimate.z']

    FINAL_ARRAY2.append(dic_temp)
   
###Speed Callback####
def velCallback2(timestamp, data, logconf):

    global FINAL_ARRAY2
    dic_temp = data
    dic_temp.update({"time": timestamp})

    global VX2
    global VY2
    global VZ2

    VX2 = data['stateEstimate.vx']
    VY2 = data['stateEstimate.vy']
    VZ2 = data['stateEstimate.vz']

    FINAL_ARRAY2.append(dic_temp)

###Quaternion Callback####
def quaternionCallback2(timestamp, data, logconf):

    global FINAL_ARRAY2
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY2.append(dic_temp)
    
##Angles Rate Callback####
def anglesRatesCallback2(timestamp, data, logconf):

    global FINAL_ARRAY2
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY2.append(dic_temp)

#############END OF CALLBACKS SECTION FOR DRONES##############################

# Function to estimate the position of the drone in question
def wait_for_position_estimator(scf):
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

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

#Function to indicate the initial position of the drone and estimate the positions in a better way
def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

#Function to reset the position estimator of the drone in question
def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

#Function that will be anchored to THREAD 1 for drone 1, speed control with potential fields.
def run_sequence(scf, base_x, base_y, base_z, yaw, logconf_pos_ang, logconf_vel, logconf_quat, logconf_angles_rates):

    cf = scf.cf

    #All log configurations created for drone 1 are added
    cf.log.add_config(logconf_pos_ang)
    cf.log.add_config(logconf_vel)
    cf.log.add_config(logconf_quat)
    cf.log.add_config(logconf_angles_rates)

    #The functions that will serve as callbacks for drone 1 are anchored
    logconf_pos_ang.data_received_cb.add_callback(pos_angCallback)
    logconf_vel.data_received_cb.add_callback(velCallback)
    logconf_quat.data_received_cb.add_callback(quaternionCallback)
    logconf_angles_rates.data_received_cb.add_callback(anglesRatesCallback)

    #Logs and data collection for drone 1 are started
    logconf_pos_ang.start()
    logconf_vel.start()
    logconf_quat.start()
    logconf_angles_rates.start()

    #The position and speed variables of drone 1 are taken.
    global X1
    global Y1
    global Z1
    global VX1
    global VY1
    global VZ1

    #The position and speed variables of its neighbor in this case drone 2.
    global X2
    global Y2


    #Speed control constants for drone 1
    kpx = 1.0
    kdx = 0.16

    kpy = 1.0
    kdy = 0.16

    kpz = 0.5 
    kdz = 0.03 

    #The desired points are taken for drone 1 calculated in the virtual leader function (they are constantly updated)
    global puntoDeseado_x_dron1
    global puntoDeseado_y_dron1
    global puntoDeseado_z_dron1
    #The flag is taken to know when the leader has finished and to end the thread
    global liderAunCorreFlag

    #These positions are assigned to the desired points for the control algorithm.
    x_deseado = puntoDeseado_x_dron1
    y_deseado = puntoDeseado_y_dron1
    z_deseado = puntoDeseado_z_dron1
    
    error_tolerado = 0.12 #Tolerated error for speed control algorithm
    contador = 1 #Counter to know the iteration in which we are going (it does not affect the control in any way, it can be removed) 
    saturacion_error = 0.5 # Maximum saturation allowed for the resulting velocities calculated in X, Y, Z

    #Gains for potential fields and safety distance
    kp_potencial = 1.0
    kt_potencial = 70 
    kr_potencial = 1   
    distancia_seguridad = 0.40
    #############################
    

    print("Iteracion: " + str(contador))

    #Main while loop to know when to end the code
    while liderAunCorreFlag:

        #The desired points are taken again on the three axes
        x_deseado = puntoDeseado_x_dron1
        y_deseado = puntoDeseado_y_dron1
        z_deseado = puntoDeseado_z_dron1

        #*Estimated error
        error_x = abs(X1 - x_deseado)
        error_y = abs(Y1 - y_deseado)
        error_z = abs(Z1 - z_deseado)

        #While cycle for speed control algorithm using position error
        while ( (error_x > error_tolerado) or (error_y > error_tolerado) or (error_z > error_tolerado) ):
                
            #Calculation to control drones by speed
            vx_send = -kpx*(X1 - x_deseado) - kdx*(VX1)
            vy_send = -kpy*(Y1 - y_deseado) - kdy*(VY1)
            vz_send = -kpz*(Z1 - z_deseado) - kdz*(VZ1)


            ###################################POTENTIAL FIELDS######################################################

            #The vectors necessary for the calculations of potential fields are calculated
            punto_deseado_dron1 = np.array([[x_deseado],[y_deseado]])
            punto_actual_dron1 = np.array([[X1],[Y1]])
            punto_vecino = np.array([[X2],[Y2]])
            norma_pv_pa = np.linalg.norm(np.subtract(punto_vecino, punto_actual_dron1))
            
            #The formula is divided into three sections
            vel_campos_potenciales_parte_izq = -kp_potencial*(np.subtract(punto_actual_dron1, punto_deseado_dron1))
            vel_campos_potenciales_parte_der = -(np.subtract(punto_vecino, punto_actual_dron1) / norma_pv_pa)
            coeficienteDeControl_parte_der = (-0.5*np.tanh(kt_potencial*(norma_pv_pa - distancia_seguridad)) + 0.5) * (kr_potencial / norma_pv_pa)

            #The three sections are put together to obtain the vector with the resulting velocities
            velocidades_resultantes_campos_potenciales = vel_campos_potenciales_parte_izq + vel_campos_potenciales_parte_der * coeficienteDeControl_parte_der

            #The resulting values ​​of the potential fields for the X, Y axes are obtained
            vx_campo_potencial = velocidades_resultantes_campos_potenciales[0,0]
            vy_campo_potencial = velocidades_resultantes_campos_potenciales[1,0]

            #The speed that had been previously calculated is added to what was calculated in the potential fields
            vx_send = vx_send + vx_campo_potencial
            vy_send = vy_send + vy_campo_potencial

            ###################################################################################

            #mSATURATIONS of speeds##############################################################

            if(vx_send > saturacion_error):
                vx_send = saturacion_error
            elif(vx_send < -saturacion_error):
                vx_send = -saturacion_error

            if(vy_send > saturacion_error):
                vy_send = saturacion_error
            elif(vy_send < -saturacion_error):
                vy_send = -saturacion_error
                
            if(vz_send > saturacion_error):
                vz_send = saturacion_error
            elif(vz_send < -saturacion_error):
                vz_send = -saturacion_error
            
            ###########END SATURATIONS of speeds#########################################################

            #Calculated speeds are sent
            cf.commander.send_velocity_world_setpoint(vx_send, vy_send, vz_send, 0.0) #vx, vy, vz, yawRate
            time.sleep(0.1)

            #error calculations
            error_x = abs(X1 - x_deseado)
            error_y = abs(Y1 - y_deseado)
            error_z = abs(Z1 - z_deseado)

            #The next desired point is obtained again
            x_deseado = puntoDeseado_x_dron1
            y_deseado = puntoDeseado_y_dron1
            z_deseado = puntoDeseado_z_dron1

            
    contador+=1


    #End of the logs
    logconf_pos_ang.stop()
    logconf_vel.stop()
    logconf_quat.stop()
    logconf_angles_rates.stop()

    #Motors stop
    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

    #Excel is generated with the data collected from the drone
    global FINAL_ARRAY
    df = pandas.DataFrame(FINAL_ARRAY)
    df.to_csv("./campos/06_06_23_Prueba029_3_LiderConCampos_drone1.csv")


#Function that will be anchored to THREAD 2 for drone 2, speed control with potential fields.
#NOTE: this function is not commented because it works exactly the same as the function of drone 1, it only changes
#That the positions, speeds etc... of drone 2 are taken, but they work the same.
def run_sequence2(scf, base_x, base_y, base_z, yaw, logconf_pos_ang, logconf_vel, logconf_quat, logconf_angles_rates):
    cf = scf.cf

    cf.log.add_config(logconf_pos_ang)
    cf.log.add_config(logconf_vel)
    cf.log.add_config(logconf_quat)
    cf.log.add_config(logconf_angles_rates)

    logconf_pos_ang.data_received_cb.add_callback(pos_angCallback2)
    logconf_vel.data_received_cb.add_callback(velCallback2)
    logconf_quat.data_received_cb.add_callback(quaternionCallback2)
    logconf_angles_rates.data_received_cb.add_callback(anglesRatesCallback2)


    logconf_pos_ang.start()
    logconf_vel.start()
    logconf_quat.start()
    logconf_angles_rates.start()

    
    global X2
    global Y2
    global Z2
    global VX2
    global VY2
    global VZ2

    kpx = 1.0
    kdx = 0.16

    kpy = 1.0
    kdy = 0.16

    kpz = 0.5 
    kdz = 0.03 

    global puntoDeseado_x_dron2
    global puntoDeseado_y_dron2
    global puntoDeseado_z_dron2
    global liderAunCorreFlag

    x_deseado = puntoDeseado_x_dron2
    y_deseado = puntoDeseado_y_dron2
    z_deseado = puntoDeseado_z_dron2

    error_tolerado = 0.12
    contador = 1

    ##Gains potential fields
    kp_potencial = 1.0
    kt_potencial = 70 
    kr_potencial = 1   
    distancia_seguridad = 0.40
    #############################

    saturacion_error = 0.5
    while liderAunCorreFlag:



        x_deseado = puntoDeseado_x_dron2
        y_deseado = puntoDeseado_y_dron2
        z_deseado = puntoDeseado_z_dron2

        error_x = abs(X2 - x_deseado)
        error_y = abs(Y2 - y_deseado)
        error_z = abs(Z2 - z_deseado)
        

        print("Iteracion: " + str(contador))
        while ( (error_x > error_tolerado) or (error_y > error_tolerado) or (error_z > error_tolerado) ):
            
            

            if kb.is_pressed("q"):
                print("q")
                break

            vx_send = -kpx*(X2 - x_deseado) - kdx*(VX2)
            vy_send = -kpy*(Y2 - y_deseado) - kdy*(VY2)
            vz_send = -kpz*(Z2 - z_deseado) - kdz*(VZ2)

            ##########POTENTIAL FIELDS######################################################


            punto_deseado_dron1 = np.array([[x_deseado],[y_deseado]])
            punto_actual_dron1 = np.array([[X2],[Y2]])
            punto_vecino = np.array([[X1],[Y1]])
            norma_pv_pa = np.linalg.norm(np.subtract(punto_vecino, punto_actual_dron1))
            

            vel_campos_potenciales_parte_izq = -kp_potencial*(np.subtract(punto_actual_dron1, punto_deseado_dron1))
            vel_campos_potenciales_parte_der = -(np.subtract(punto_vecino, punto_actual_dron1) / norma_pv_pa)
            coeficienteDeControl_parte_der = (-0.5*np.tanh(kt_potencial*(norma_pv_pa - distancia_seguridad)) + 0.5) * (kr_potencial / norma_pv_pa)

            velocidades_resultantes_campos_potenciales = vel_campos_potenciales_parte_izq + vel_campos_potenciales_parte_der * coeficienteDeControl_parte_der

            vx_campo_potencial = velocidades_resultantes_campos_potenciales[0,0]
            vy_campo_potencial = velocidades_resultantes_campos_potenciales[1,0]

            vx_send = vx_send + vx_campo_potencial
            vy_send = vy_send + vy_campo_potencial

            ###################################################################################

            #SATURATIONS

            if(vx_send > saturacion_error):
                vx_send = saturacion_error
            elif(vx_send < -saturacion_error):
                vx_send = -saturacion_error

            if(vy_send > saturacion_error):
                vy_send = saturacion_error
            elif(vy_send < -saturacion_error):
                vy_send = -saturacion_error
            
            if(vz_send > saturacion_error):
                vz_send = saturacion_error
            elif(vz_send < -saturacion_error):
                vz_send = -saturacion_error
        
            #Calculated speeds are sent
            cf.commander.send_velocity_world_setpoint(vx_send, vy_send, vz_send, 0.0) #vx, vy, vz, yawRate
            time.sleep(0.1)

            error_x = abs(X2 - x_deseado)
            error_y = abs(Y2 - y_deseado)
            error_z = abs(Z2 - z_deseado)

            x_deseado = puntoDeseado_x_dron2
            y_deseado = puntoDeseado_y_dron2
            z_deseado = puntoDeseado_z_dron2

    
    contador+=1



    logconf_pos_ang.stop()
    logconf_vel.stop()
    logconf_quat.stop()
    logconf_angles_rates.stop()

    cf.commander.send_stop_setpoint()
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing

    time.sleep(0.1)

    global FINAL_ARRAY2
    df = pandas.DataFrame(FINAL_ARRAY2)
    df.to_csv("./campos/06_06_23_Prueba029_3_LiderConCampos_drone2.csv")

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    #Initial Drone 1 Positions
    initial_x = puntoDeseado_x_dron1 
    initial_y = puntoDeseado_y_dron1 
    initial_z = 0.0
    initial_yaw = 90  # en grados

    #Initial Drone 2 Positions
    initial_x2 = puntoDeseado_x_dron2
    initial_y2 = puntoDeseado_y_dron2
    initial_z2 = 0.0
    initial_yaw2 = 90
    

    #####LOG CONFIGURATION ####################################
    #####Position and angle variables########################################################
    lg_stab_pos_ang = LogConfig(name='Stabilizer1', period_in_ms=100)

    lg_stab_pos_ang.add_variable('stabilizer.roll', 'float')
    lg_stab_pos_ang.add_variable('stabilizer.pitch', 'float')
    lg_stab_pos_ang.add_variable('stabilizer.yaw', 'float')
    
    lg_stab_pos_ang.add_variable('stateEstimate.x', 'float')
    lg_stab_pos_ang.add_variable('stateEstimate.y', 'float')
    lg_stab_pos_ang.add_variable('stateEstimate.z', 'float')
#############Speeds in x,y,z#####################################################################

    lg_stab_vel = LogConfig(name='Stabilizer2', period_in_ms=100)
    
    lg_stab_vel.add_variable('stateEstimate.vx', 'float')
    lg_stab_vel.add_variable('stateEstimate.vy', 'float')
    lg_stab_vel.add_variable('stateEstimate.vz', 'float')

###################### QUATERNIONS PART#############################################################################

    lg_stab_quat = LogConfig(name='Stabilizer3', period_in_ms=100)
    
    lg_stab_quat.add_variable('stateEstimate.qx', 'float')
    lg_stab_quat.add_variable('stateEstimate.qy', 'float')
    lg_stab_quat.add_variable('stateEstimate.qz', 'float')
    lg_stab_quat.add_variable('stateEstimate.qw', 'float')

###################################################################################################

    lg_stab_angles_rates = LogConfig(name='Stabilizer4', period_in_ms=100)
    
    lg_stab_angles_rates.add_variable('controller.r_pitch', 'float')
    lg_stab_angles_rates.add_variable('controller.r_roll', 'float')
    lg_stab_angles_rates.add_variable('controller.r_yaw', 'float')

##############################################################################################
####################Instances for drone 2##################################################
    lg_stab_pos_ang2 = LogConfig(name='Stabilizer5', period_in_ms=100)

    lg_stab_pos_ang2.add_variable('stabilizer.roll', 'float')
    lg_stab_pos_ang2.add_variable('stabilizer.pitch', 'float')
    lg_stab_pos_ang2.add_variable('stabilizer.yaw', 'float')
    
    lg_stab_pos_ang2.add_variable('stateEstimate.x', 'float')
    lg_stab_pos_ang2.add_variable('stateEstimate.y', 'float')
    lg_stab_pos_ang2.add_variable('stateEstimate.z', 'float')
#############Speeds in x,y,z#####################################################################

    lg_stab_vel2 = LogConfig(name='Stabilizer6', period_in_ms=100)
    
    lg_stab_vel2.add_variable('stateEstimate.vx', 'float')
    lg_stab_vel2.add_variable('stateEstimate.vy', 'float')
    lg_stab_vel2.add_variable('stateEstimate.vz', 'float')

###################### QUATERNIONS PART#############################################################################

    lg_stab_quat2 = LogConfig(name='Stabilizer7', period_in_ms=100)
    
    lg_stab_quat2.add_variable('stateEstimate.qx', 'float')
    lg_stab_quat2.add_variable('stateEstimate.qy', 'float')
    lg_stab_quat2.add_variable('stateEstimate.qz', 'float')
    lg_stab_quat2.add_variable('stateEstimate.qw', 'float')

###################################################################################################

    lg_stab_angles_rates2 = LogConfig(name='Stabilizer8', period_in_ms=100)
    
    lg_stab_angles_rates2.add_variable('controller.r_pitch', 'float')
    lg_stab_angles_rates2.add_variable('controller.r_roll', 'float')
    lg_stab_angles_rates2.add_variable('controller.r_yaw', 'float')



################################################################################################

    #Connection with drones 
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with SyncCrazyflie(uri2, cf=Crazyflie(rw_cache='./cache')) as scf2:

        
            #Indicate initial position of the drones and restart drone estimators
            set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
            reset_estimator(scf)

            set_initial_position(scf2, initial_x2, initial_y2, initial_z2, initial_yaw2)
            reset_estimator(scf2)

            #Drone 1 thread is created
            hilo_dron1 = threading.Thread(target=run_sequence, args=(scf, initial_x, initial_y, initial_z, initial_yaw,
                                                                    lg_stab_pos_ang, lg_stab_vel, lg_stab_quat, lg_stab_angles_rates))

            #Drone 2 thread is created
            hilo_dron2 = threading.Thread(target=run_sequence2, args=(scf2, initial_x2, initial_y2, initial_z2, initial_yaw2,
                                                                    lg_stab_pos_ang2, lg_stab_vel2, lg_stab_quat2, lg_stab_angles_rates2))

            #Both threads are executed, both drones start flying
            hilo_dron1.start()
            hilo_dron2.start()

            #Start function of the virtual leader WITHOUT THREAD so that it is in the main thread#########################################
            liderVirtual()
            ##########################################################################

            
            #Wait for the threads to finish to finish the code
            hilo_dron1.join()
            hilo_dron2.join()
