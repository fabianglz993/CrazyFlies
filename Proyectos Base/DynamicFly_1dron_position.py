#################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 3
# Fecha de última modificación: Semana 4
# Descripción: Código para mandar el dron a una coordenada (x,y) y mantenerse a una altura de 1 metro 
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Adrian Lara Guzman


#Código basado en https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/positioning/initial_position.py
#  Copyright (C) 2019 Bitcraze AB

#################################################################################################################
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

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')

# Change the sequence according to your setup
#             x    y    z
sequence = [
    (0, 0, 1), #Center
    (0.3, -0.3, 1) #Point coordinates
]

##################################Function to implement tha Kalman filter##########################################################
def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')
    
    # Configure logging for the Kalman filter variance
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
        for log_entry in logger:
            data = log_entry[1]
            # Update the variance history lists.
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

###############Function that receives the initial positions of the drone and uses the Kalman filter########################################
def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

###################Function that resets the previous values ​​stored in the Kalman filter#######################################
def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    #Calls the next function
    wait_for_position_estimator(cf)

#################Function that calls the sequence of point that the drone has to follow##################################################
def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):
    cf = scf.cf

    for position in sequence:
        print('Setting position {}'.format(position))

        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z

        for i in range(50):
            cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Set these to the position and yaw based on how your Crazyflie is placed on the floor
    initial_x = 2.4
    initial_y = 3.15
    initial_z = 0.0
    initial_yaw = 90  # In degrees
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    #####################Connect to the drone####################################################
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        reset_estimator(scf)
        run_sequence(scf, sequence,
                     initial_x, initial_y, initial_z, initial_yaw)
        
       
