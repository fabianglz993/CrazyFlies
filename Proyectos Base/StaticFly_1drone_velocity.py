################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 7
# Fecha de última modificación: Semana 8
# Descripción: Código para mantener un vuelo de un dron, en una altura de 1 metro y una X y Y fijas con control de velocidades
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Angel Sanchez 

#  Copyright (C) 2019 Bitcraze AB

#################################################################################################################
#Import standar libraries
import math
import time
import keyboard as kb
import threading
import pandas

#Import libraries to communicate with Crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper


#Global variables to obtain the position of drone number 1.
X1 = 0
Y1 = 0
Z1 = 0

VX1 = 0
VY1 = 0
VZ1 = 0

#Global variables to obtain the position of drone number 2.
X2 = 0
Y2 = 0
Z2 = 0

VX2 = 0
VY2 = 0
VZ2 = 0


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703')
uri2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')
# Change the sequence according to your setup
#             x    y    z

#Arrays where the recolected data will be stored while the code is running.
FINAL_ARRAY = []
FINAL_ARRAY2 = []

#############Drone 1 callbacks###################

###Postition Callback####
def pos_angCallback(timestamp, data, logconf):
    #Esta funcion nos permite obtener el angulo (yaw) para poder hacer los logs
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
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
    #Esta funcion nos permite obtener las velocidades para poder hacer los logs
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
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
    #Esta funcion nos permite obtener los quaterniones para poder hacer los logs
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)

##Angles Rate Callback####
def anglesRatesCallback(timestamp, data, logconf):
    #Esta funcion nos permite obtener las AR para poder hacer los logs

    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)

#############Drone 2 callbacks###################
###Postition Callback####
def pos_angCallback2(timestamp, data, logconf):
    #Esta funcion nos permite obtener la posicion angular del dron 2 para poder hacer los logs
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
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
    #Esta funcion nos permite obtener las velocidades del dron 2 para poder hacer los logs
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
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
    #Esta funcion nos permite obtener los quaterniones del dron 2 para poder hacer los logs
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY2
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY2.append(dic_temp)
    
##Angles Rate Callback####
def anglesRatesCallback2(timestamp, data, logconf):
    #Esta funcion nos permite obtener los AR del dron 2 para poder hacer los logs
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY2
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY2.append(dic_temp)

##################################Function to implement the Kalman Filter##########################################################
def wait_for_position_estimator(scf):
    # Filtro predictor
    print('Waiting for estimator to find position...')

    # Configuration for logging Kalman variance
    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    # History buffers for variance values
    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    # Threshold for stable variance values
    threshold = 0.001

    # Using SyncLogger to capture log data from the Crazyflie
    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            # Update variance history buffers
            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            # Calculate min and max values for each axis
            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # Check if the variance values are below the threshold
            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break    

###############Function that recives the initial positions of the dron and uses the kalman filter####################################
def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

###################Function that restarts the values previously stored in the Kalman filter#######################################
def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

#################Function that calls the sequence of the points that the drone  has to follow##################################################
def run_sequence(scf, base_x, base_y, base_z, yaw, logconf_pos_ang, logconf_vel, logconf_quat, logconf_angles_rates):
    cf = scf.cf

    # Logs configuration
    cf.log.add_config(logconf_pos_ang)
    cf.log.add_config(logconf_vel)
    cf.log.add_config(logconf_quat)
    cf.log.add_config(logconf_angles_rates)

    # Callbacks for handling received log data
    logconf_pos_ang.data_received_cb.add_callback(pos_angCallback)
    logconf_vel.data_received_cb.add_callback(velCallback)
    logconf_quat.data_received_cb.add_callback(quaternionCallback)
    logconf_angles_rates.data_received_cb.add_callback(anglesRatesCallback)

    # Start logging
    logconf_pos_ang.start()
    logconf_vel.start()
    logconf_quat.start()
    logconf_angles_rates.start()

    # Global variables for position and velocity
    global X1, Y1, Z1, VX1, VY1, VZ1

    # Controller gains for the velocity controller
    kpx = 1.0
    kdx = 0.16
    kpy = 1.0
    kdy = 0.16
    kpz = 0.5
    kdz = 0.03

    # Desired initial position
    x_deseado = base_x
    y_deseado = base_y
    z_deseado = 1.0

    # Points to visit in the sequence
    points = [
        [base_x, base_y, 1], #Center
        [base_x + 0.7, base_y + 0.7, 1], #Upper left corner
        [base_x + 0.7, base_y - 0.7, 1], #Upper right corner
        [base_x - 0.7, base_y - 0.7, 1], #Bottom right corner
        [base_x - 0.7, base_y + 0.7, 1], #Bottom left corner
        [base_x, base_y, 1], #Center
        [base_x, base_y, 0.0],
        [base_x, base_y, 0.0] #Landing Center
    ]

    error_tolerado = 0.10
    contador = 1

    saturacion_error = 0.8

    for point in points:
        # Implementation of the velocity control

        x_deseado = point[0]
        y_deseado = point[1]
        z_deseado = point[2]

        print(x_deseado, y_deseado, z_deseado)

        error_x = abs(X1 - x_deseado)
        error_y = abs(Y1 - y_deseado)
        error_z = abs(Z1 - z_deseado)

        print("Iteracion: " + str(contador))

        # Velocity control loop
        while (error_x > error_tolerado) or (error_y > error_tolerado) or (error_z > error_tolerado):
            # Break the loop if the 'q' key is pressed
            if kb.is_pressed("q"):
                print("q")
                break

            # PD controller for velocity
            vx_send = -kpx * (X1 - x_deseado) - kdx * (VX1)
            vy_send = -kpy * (Y1 - y_deseado) - kdy * (VY1)
            vz_send = -kpz * (Z1 - z_deseado) - kdz * (VZ1)

            # Saturations
            vx_send = max(min(vx_send, saturacion_error), -saturacion_error)
            vy_send = max(min(vy_send, saturacion_error), -saturacion_error)
            vz_send = max(min(vz_send, saturacion_error), -saturacion_error)

            # Send the calculated velocities
            cf.commander.send_velocity_world_setpoint(vx_send, vy_send, vz_send, 0.0)
            time.sleep(0.1)

            # Update errors
            error_x = abs(X1 - x_deseado)
            error_y = abs(Y1 - y_deseado)
            error_z = abs(Z1 - z_deseado)

        contador += 1

    # Stop logging and reset the drone
    logconf_pos_ang.stop()
    logconf_vel.stop()
    logconf_quat.stop()
    logconf_angles_rates.stop()
    cf.commander.send_stop_setpoint()
    time.sleep(0.1)

    # Save the final array to a CSV file
    global FINAL_ARRAY
    df = pandas.DataFrame(FINAL_ARRAY)
    df.to_csv("./logs2_drone1/23_05_23_011_VueloCuadradoHilos_drone1.csv")

###################################################################
def run_sequence2(scf, base_x, base_y, base_z, yaw, logconf_pos_ang, logconf_vel, logconf_quat, logconf_angles_rates):
    cf = scf.cf

    # Logs configuration for drone 2
    cf.log.add_config(logconf_pos_ang)
    cf.log.add_config(logconf_vel)
    cf.log.add_config(logconf_quat)
    cf.log.add_config(logconf_angles_rates)

    # Callbacks for handling received log data for drone 2
    logconf_pos_ang.data_received_cb.add_callback(pos_angCallback2)
    logconf_vel.data_received_cb.add_callback(velCallback2)
    logconf_quat.data_received_cb.add_callback(quaternionCallback2)
    logconf_angles_rates.data_received_cb.add_callback(anglesRatesCallback2)

    # Start logging for drone 2
    logconf_pos_ang.start()
    logconf_vel.start()
    logconf_quat.start()
    logconf_angles_rates.start()

    # Global variables for position and velocity of drone 2
    global X2, Y2, Z2, VX2, VY2, VZ2

    # Controller gains for the velocity controller of drone 2
    kpx = 1.0
    kdx = 0.16
    kpy = 1.0
    kdy = 0.16
    kpz = 0.5
    kdz = 0.03

    # Desired initial position for drone 2
    x_deseado = base_x
    y_deseado = base_y
    z_deseado = 1.0

    # Points to visit in the sequence for drone 2
    points = [
        [base_x, base_y, 1],- #Center
        [base_x + 0.7, base_y + 0.7, 1], #Upper left corner
        [base_x + 0.7, base_y - 0.7, 1], #Upper right corner
        [base_x - 0.7, base_y - 0.7, 1], #Bottom right corner
        [base_x - 0.7, base_y + 0.7, 1], #Bottom left corner
        [base_x, base_y, 1], #Center
        [base_x, base_y, 0.0],
        [base_x, base_y, 0.0] #Landing Center
    ]

    error_tolerado = 0.10
    contador = 1

    saturacion_error = 0.8

    for point in points:
        # Implementation of the velocity control for drone 2

        x_deseado = point[0]
        y_deseado = point[1]
        z_deseado = point[2]

        print(x_deseado, y_deseado, z_deseado)

        error_x = abs(X2 - x_deseado)
        error_y = abs(Y2 - y_deseado)
        error_z = abs(Z2 - z_deseado)

        print("Iteracion: " + str(contador))

        # Velocity control loop for drone 2
        while (error_x > error_tolerado) or (error_y > error_tolerado) or (error_z > error_tolerado):
            # Break the loop if the 'q' key is pressed
            if kb.is_pressed("q"):
                print("q")
                break

            # PD controller for velocity for drone 2
            vx_send = -kpx * (X2 - x_deseado) - kdx * (VX2)
            vy_send = -kpy * (Y2 - y_deseado) - kdy * (VY2)
            vz_send = -kpz * (Z2 - z_deseado) - kdz * (VZ2)

            # Saturations for drone 2
            vx_send = max(min(vx_send, saturacion_error), -saturacion_error)
            vy_send = max(min(vy_send, saturacion_error), -saturacion_error)
            vz_send = max(min(vz_send, saturacion_error), -saturacion_error)

            # Send the calculated velocities for drone 2
            cf.commander.send_velocity_world_setpoint(vx_send, vy_send, vz_send, 0.0)
            time.sleep(0.1)

            # Update errors for drone 2
            error_x = abs(X2 - x_deseado)
            error_y = abs(Y2 - y_deseado)
            error_z = abs(Z2 - z_deseado)

        contador += 1

    # Stop logging and reset the drone 2
    logconf_pos_ang.stop()
    logconf_vel.stop()
    logconf_quat.stop()
    logconf_angles_rates.stop()
    cf.commander.send_stop_setpoint()
    time.sleep(0.1)

    # Save the final array to a CSV file for drone


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Initial position of both drones
    initial_x = 1.47 
    initial_y = 1.5 #3.15
    initial_z = 0.0
    initial_yaw = 90  # In degrees

    initial_x2 = 1.47
    initial_y2 = 4.7
    initial_z2 = 0.0
    initial_yaw2 = 90
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    #####################################Postion variables and angles##################################
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
####################Instancias para el dron 2##################################################
    lg_stab_pos_ang2 = LogConfig(name='Stabilizer5', period_in_ms=100)

    lg_stab_pos_ang2.add_variable('stabilizer.roll', 'float')
    lg_stab_pos_ang2.add_variable('stabilizer.pitch', 'float')
    lg_stab_pos_ang2.add_variable('stabilizer.yaw', 'float')
    
    lg_stab_pos_ang2.add_variable('stateEstimate.x', 'float')
    lg_stab_pos_ang2.add_variable('stateEstimate.y', 'float')
    lg_stab_pos_ang2.add_variable('stateEstimate.z', 'float')
#############Velocidades en x,y,z#####################################################################

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

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with SyncCrazyflie(uri2, cf=Crazyflie(rw_cache='./cache')) as scf2:
    
            # Set initial position and reset estimator for drone 1
            set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
            reset_estimator(scf)
    
            # Set initial position and reset estimator for drone 2
            set_initial_position(scf2, initial_x2, initial_y2, initial_z2, initial_yaw2)
            reset_estimator(scf2)
    
            # Create threads for running sequences for drone 1 and drone 2
            hilo_dron1 = threading.Thread(target=run_sequence, args=(scf, initial_x, initial_y, initial_z, initial_yaw,
                                                                    lg_stab_pos_ang, lg_stab_vel, lg_stab_quat, lg_stab_angles_rates))
    
            hilo_dron2 = threading.Thread(target=run_sequence2, args=(scf2, initial_x2, initial_y2, initial_z2, initial_yaw2,
                                                                    lg_stab_pos_ang2, lg_stab_vel2, lg_stab_quat2, lg_stab_angles_rates2))
    
            # Start the threads for drone 1 and drone 2
            hilo_dron1.start()
            hilo_dron2.start()
    
            # Code for the leader (not provided in the snippet)
            # ...
    
            # Wait for both threads to finish before proceeding
            hilo_dron1.join()
            hilo_dron2.join()

        
