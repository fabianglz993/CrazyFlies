##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 7  
# Fecha de última modificación: Semana 8
# Descripción: Código para volar 2 drones en modo estático, el LPS debe estar en TDoA2
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Angel Sanchez
# Correciones de comentarios por: Natalia Rodríguez González

##################################################################################################################################################
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
uri1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')

uri2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702') #Uris modificadas en CfClient

# Change the sequence according to your setup
#             x    y    z
#The coordinates are absolute to the position of the drone
sequence1 = [
    (0, 0, 1),
    (0, 0, 1),
    (0, 0, 1),
    (0, 0, 0.2),
]

sequence2 = [
    (0, 0, 1),
    (0, 0, 1),
    (0, 0, 1),
    (0, 0, 0.2),
]

###############Kalman filter#######################################
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

##########Function that recives the initial positions of the dron and uses the kalman filter#######################
def set_initial_position(scf, x, y, z, yaw_deg): #Esta funcion servirá para ambos drones al igual que la de reset
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

############Function that restarts the values previously stored in the Kalman filter###################
def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

#########Function that calls the sequence of the points that the drones have to follow#########################
def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):
    cf = scf.cf

    # Iterate through each position in the sequence
    for position in sequence:
        print('Setting position {}'.format(position))

        # Adjust the position based on the base coordinates
        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z

        # Send the position setpoint to the drone for 50 iterations
        for i in range(50):
            cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    # Send a stop setpoint to the drone
    cf.commander.send_stop_setpoint()

    # Allow time for the last packet to leave before closing the link
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Initial positions of both drones
    initial_x1 = 1.475
    initial_y1 = 4.725
    initial_z1 = 0.0
    initial_yaw1 = 90  # In degrees

    initial_x2 = 1.475
    initial_y2 = 1.575
    initial_z2 = 0.0
    initial_yaw2 = 90  # In degrees
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    ####First Drone#################
    # Connect to the Crazyflie drone 1 with URI uri1
    with SyncCrazyflie(uri1, cf=Crazyflie(rw_cache='./cache')) as scf:
        # Set the initial position and yaw of the drone
        set_initial_position1(scf, initial_x1, initial_y1, initial_z1, initial_yaw1)
        # Reset the drone's estimator
        reset_estimator(scf)
        # Output a message indicating that sequence 1 is about to be executed
        print('Running sequence 1')
        # Execute the sequence of positions for the drone
        run_sequence1(scf, sequence1, initial_x1, initial_y1, initial_z1, initial_yaw1)

        
    ######Second Drone###############  
    # Connect to the Crazyflie drone 2 with URI uri2
    with SyncCrazyflie(uri2, cf=Crazyflie(rw_cache='./cache')) as scf2: 
        # Set the initial position and yaw of the drone
        set_initial_position2(scf2, initial_x2, initial_y2, initial_z2, initial_yaw2)
        # Reset the drone's estimator
        reset_estimator(scf2)
        # Output a message indicating that sequence 2 is about to be executed
        print('Running Sequence 2')
        # Execute the sequence of positions for the drone
        run_sequence2(scf2, sequence2, initial_x2, initial_y2, initial_z2, initial_yaw2)
    
