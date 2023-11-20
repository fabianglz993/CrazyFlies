##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 8
# Fecha de última modificación: Semana 8
# Descripción: Código para realizar una secuencia de cuadrado a una altura de un metro con el dron Crazyflie implementando un filtro de Kalman 
# para las señales recibidas del sistema Loco para la estimación de la posición realizando un control mediante velocidades
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Misael Cruz
# Correciones de comentarios por: Natalia Rodríguez González

##################################################################################################################################################

#  Copyright (C) 2019 Bitcraze AB
#Import standar libraries
import math
import time
import keyboard as kb
import pandas

#Import libraries to communicate with Crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper


#Postiton global variables
X1 = 0
Y1 = 0
Z1 = 0
#Speed global  variables
VX1 = 0
VY1 = 0
VZ1 = 0

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

# Change the sequence according to your setup
#             x    y    z

#The array where the recolected data will be stored while the code is running.
FINAL_ARRAY = []

#######################Postition Callback################################
def pos_Callback(timestamp, data, logconf):
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    global X1
    global Y1
    global Z1

 #   print("ES DATA")
  #  print(data)

    X1 = data['stateEstimate.x']
    Y1 = data['stateEstimate.y']
    Z1 = data['stateEstimate.z']

    FINAL_ARRAY.append(dic_temp)

#############################Function to update the Array#####################################
def dataExcelLogs(timestamp, data, logconf):
    # Uncomment the following line if you want to print the timestamp, log name, and data.
    # print('[%d][%s]: %s' % (timestamp, logconf.name, data))

    # Access the global array to store the logged data.
    global FINAL_ARRAY
    
    # Create a temporary dictionary with timestamp as 'time' and additional logged data.
    dic_temp = data
    dic_temp.update({"time": timestamp})

    # Access the global variables to store velocity components.
    global VX1
    global VY1
    global VZ1

    # Update the global velocity variables with the current values from the log data.
    VX1 = data['stateEstimate.vx']
    VY1 = data['stateEstimate.vy']
    VZ1 = data['stateEstimate.vz']
    
    # Append the temporary dictionary to the global array.
    FINAL_ARRAY.append(dic_temp)
    
#############Function to implement a Kalman filter o optimally estimate the variables of interests when they can't be measured directly#####################
def wait_for_position_estimator(scf):
    # Print a message indicating that the code is waiting for the position estimator to find the position.
    print('Waiting for estimator to find position...')

    # Configure logging for the Kalman filter variance with a specified logging period.
    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    # Initialize history lists for variance values for X, Y, and Z axes.
    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    # Set a threshold for the difference between maximum and minimum variance values.
    threshold = 0.001

    # Use a SyncLogger to capture logging data from the Crazyflie.
    with SyncLogger(scf, log_config) as logger:
        # Iterate over the logged entries.
        for log_entry in logger:
            # Extract the data from the log entry.
            data = log_entry[1]

            # Update the variance history lists, keeping a window of the last 10 values.
            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            # Calculate the minimum and maximum variance values for each axis.
            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # Uncomment the following line if you want to print the differences between max and min values.
            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            # Check if the differences between max and min values for all axes are below the threshold.
            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                # If the condition is met, exit the loop and finish waiting for the position estimator.
                break

###############Function that recives the initial positions that the drone recives and uses the kalman filter#######################
def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)
###################Function that restarts the values of the kalman filter that where previously stored############################
def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

################Function that calls the sequence of the points that the drone has to follow##################################################
def run_sequence(scf, base_x, base_y, logconf, logconf2):
    # Get the Crazyflie object from the SyncCrazyflie object.
    cf = scf.cf

    # Add logging configurations to the Crazyflie.
    cf.log.add_config(logconf)
    cf.log.add_config(logconf2)
    
    # Attach callback functions to handle received log data.
    logconf.data_received_cb.add_callback(dataExcelLogs)
    logconf2.data_received_cb.add_callback(pos_Callback)
    
    # Start logging for each configuration.
    logconf.start()
    logconf2.start()
    
    # Define global variables to store position and velocity information.
    global X1
    global Y1
    global Z1
    global VX1
    global VY1
    global VZ1
    
    # Define proportional and derivative gains for the PID controller.
    kpx = 1.0
    kdx = 0.16

    kpy = 1.0
    kdy = 0.16

    kpz = 0.5  # 0.5
    kdz = 0.03  # 0.03

    # Define a sequence of points for the Crazyflie to navigate to, specified in absolute coordinates.
    # Format: [x, y, z]
    points = [
        [base_x, base_y, 1], #Center
        [base_x + 0.7, base_y + 0.7, 1], #Upper Left Corner
        [base_x + 0.7, base_y - 0.7, 1], #Upper Right Corner
        [base_x - 0.7, base_y - 0.7, 1], #Bottom Right Corner
        [base_x - 0.7, base_y + 0.7, 1], #Bottom Left Corner
        [base_x, base_y, 1], #Center
        [base_x, base_y, 0.0],
        [base_x, base_y, 0.0] #Landing Center
    ]

    # Define an error tolerance for the position control.
    error_tolerado = 0.05
    contador = 1

    # Define the maximum velocity value, m/s.
    saturacion_error = 0.5

    # Iterate over the list of points to navigate.
    for point in points:
        x_deseado = point[0]
        y_deseado = point[1]
        z_deseado = point[2]

        # Print the desired position for debugging purposes.
        print(x_deseado, y_deseado, z_deseado)

        # Calculate errors in position.
        error_x = abs(X1 - x_deseado)
        error_y = abs(Y1 - y_deseado)
        error_z = abs(Z1 - z_deseado)

        # Print iteration information for debugging purposes.
        print("Iteracion: " + str(contador))

        # Enter a control loop until the position errors are within the tolerance.
        while (error_x > error_tolerado) or (error_y > error_tolerado) or (error_z > error_tolerado):
            # As a safety measure, if the 'q' key is pressed, the program terminates.
            if kb.is_pressed("q"):
                print("q")
                break

            # Calculate velocity commands using PID control.
            vx_send = -kpx * (X1 - x_deseado) - kdx * (VX1)
            vy_send = -kpy * (Y1 - y_deseado) - kdy * (VY1)
            vz_send = -kpz * (Z1 - z_deseado) - kdz * (VZ1)

            # Saturations to limit the velocity values.
            if vx_send > saturacion_error:
                vx_send = saturacion_error
            elif vx_send < -saturacion_error:
                vx_send = -saturacion_error

            if vy_send > saturacion_error:
                vy_send = saturacion_error
            elif vy_send < -saturacion_error:
                vy_send = -saturacion_error
            
            if vz_send > saturacion_error:
                vz_send = saturacion_error
            elif vz_send < -saturacion_error:
                vz_send = -saturacion_error

            # Send the calculated speeds to the Crazyflie.
            cf.commander.send_velocity_world_setpoint(vx_send, vy_send, vz_send, 0.0)  # vx, vy, vz, yawRate
            time.sleep(0.1)

            # Update position errors.
            error_x = abs(X1 - x_deseado)
            error_y = abs(Y1 - y_deseado)
            error_z = abs(Z1 - z_deseado)

        contador += 1
        time.sleep(0.5)

        # Landing (commented out for now)
        # cf.commander.send_position_setpoint(base_x, base_y, 0.2, yaw)
        # time.sleep(0.1)

    # Sleep for 2 seconds.
    # time.sleep(2)

    # Stop logging configurations.
    logconf.stop()
    logconf2.stop()

    # Send a stop setpoint to the Crazyflie.
    cf.commander.send_stop_setpoint()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing.
    time.sleep(0.1)

    # Generate and save Excel data from the global FINAL_ARRAY.
    generateExcelData()

############################Function that generates an Excel worksheet with all the data##################################
def generateExcelData():
    global FINAL_ARRAY
    df = pandas.DataFrame(FINAL_ARRAY)
    df.to_csv("./logs_pruebas1dron_31Mayo/31_05_23_Prueba018_3_Punto3Dcon1dron_TDOA2.csv")


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Set these to the position and yaw based on how your Crazyflie is placed on the floor
    initial_x = 1.47 
    initial_y = 3.15 #3.15
    initial_z = 0.0
    initial_yaw = 90  # In degrees
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    ##############################################3Position variables and angles###############################
    # With this command we call to the variables of the drone that we want to read. The name of the variable is
    #obtained from the CFClient
    # Note: You cannot add more than 6 variables to measure for each lg_stb
    lg_stab = LogConfig(name='Stabilizer1', period_in_ms=100)

    lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('stateEstimate.vx', 'float')
    lg_stab.add_variable('stateEstimate.vy', 'float')
    lg_stab.add_variable('stateEstimate.vz', 'float')
    ########################################Pos log########################################
    lg_stab2 = LogConfig(name='Stabilizer2', period_in_ms=100)

    lg_stab2.add_variable('stateEstimate.x', 'float')
    lg_stab2.add_variable('stateEstimate.y', 'float')
    lg_stab2.add_variable('stateEstimate.z', 'float')
    #######################################################################################

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        reset_estimator(scf)
        run_sequence(scf, initial_x, initial_y, lg_stab, lg_stab2)
