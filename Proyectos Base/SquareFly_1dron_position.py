##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 3  
# Fecha de última modificación: Semana 3
# Descripción: Código para realizar una secuencia de cuadrado a una altura de un metro con el dron Crazyflie implementando un filtro de Kalman 
# para las señales recibidas del sistema Loco para la estimación de la posición.
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Misael Cruz


##################################################################################################################################################

#  Copyright (C) 2019 Bitcraze AB
#Import standar libraries
import math
import time

#Import libraries to communicate with Crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to, which changes through the CFClient
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')

# Change the sequence according to your setup
#             x    y    z
# Position in meters relative to the drone.
sequence = [
    (0, 0, 1), #Center
    (0.4, -0.4, 1), #Upper right corner
    (-0.4, -0.4, 1), #Bottom right corner 
    (-0.4, 0.4, 1), #Bottom left corner 
    (0.4, 0.4, 1), #Upper left corner 
    (0, 0, 1),    #Center
    (0, 0, 0.2),  #Bottom
]
#######################Function that estimates the position in x, y and z until the variance its lower than the establish threshold value. ##############################
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

#######################This function establishes the initial position for the Kalman filter.############################ 
def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

########################Resest the Kalman filter estimator to improve your estimation, it is only done once
def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):
    cf = scf.cf

    for position in sequence:
        print('Setting position {}'.format(position))
        #Transforms the point of the relative sequence to absolutes by adding the initial position of the drone.
        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z

        for i in range(50):
            #Sends the position to which the donre should be. 
            cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Set these to the position and yaw based on how your Crazyflie is placed on the floor
    #Positions in meters
    initial_x = 2.4
    initial_y = 3.15
    initial_z = 0.0
    initial_yaw = 90  # In degrees
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    # Establish a connection to the Crazyflie using the SyncCrazyflie context manager.
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # Set the initial position and reset the estimator for the Crazyflie.
        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        reset_estimator(scf)
        
        # Run a specified sequence on the Crazyflie.
        run_sequence(scf, sequence, initial_x, initial_y, initial_z, initial_yaw)
        
