##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 4  
# Fecha de última modificación: Semana 5
# Descripción: Código para almacenar los datos del dron mediante la configuración de los LOGS
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Cristóbal Padilla
# Correciones de comentarios por: Natalia Rodríguez González
##################################################################################################################################################

#Import standar libraries
import math
import time
import pandas

#Import libraries to communicate with Crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')


#The array where the recolected data will be stored while the code is running. 
FINAL_ARRAY = []

####################CALLBACKS for each configuration###################################################

########################Position############################
def pos_angCallback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)
   
########################Speed############################
def velCallback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)

########################Quaternions############################
def quaternionCallback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)

########################Angles Rates############################
def anglesRatesCallback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)

#################End of callback functions###########################################


#It starts to run the code where the data is stored 
def run_sequence(scf, logconf_pos_ang, logconf_vel, logconf_quat, logconf_angles_rates):

    cf = scf.cf

    #The configuration of the logs are added to the drone.
    cf.log.add_config(logconf_pos_ang)
    cf.log.add_config(logconf_vel)
    cf.log.add_config(logconf_quat)
    cf.log.add_config(logconf_angles_rates)

    #The functions that are gonaa work as callbacks are anchored.
    logconf_pos_ang.data_received_cb.add_callback(pos_angCallback)
    logconf_vel.data_received_cb.add_callback(velCallback)
    logconf_quat.data_received_cb.add_callback(quaternionCallback)
    logconf_angles_rates.data_received_cb.add_callback(anglesRatesCallback)

    #Initiates the recovery of the information for each configuration
    logconf_pos_ang.start()
    logconf_vel.start()
    logconf_quat.start()
    logconf_angles_rates.start()

    
    #For a lapse of 10 seconds the corresponding callbacks will be called y the data will be stored.
    time.sleep(10)

    #The 10 seconds end and the callbacks end.
    logconf_pos_ang.stop()
    logconf_vel.stop()
    logconf_quat.stop()
    logconf_angles_rates.stop()

    #Ends the flying the drone by turning off the motors.
    cf.commander.send_stop_setpoint()
    

    #Generates an excel worksheet with the data recolected.
    global FINAL_ARRAY
    df = pandas.DataFrame(FINAL_ARRAY)
    df.to_csv("Data.csv")


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    #Position of the drone when it starts and its orientation in yaw.
    initial_x = 1.5
    initial_y = 3.15
    initial_z = 0.0
    initial_yaw = 90  # Degrees


    ######################################################################################Configuration of the logs########################################################################################
    
    ##################Position variables and angles##############################
    lg_stab_pos_ang = LogConfig(name='Stabilizer1', period_in_ms=100)

    lg_stab_pos_ang.add_variable('stabilizer.roll', 'float')
    lg_stab_pos_ang.add_variable('stabilizer.pitch', 'float')
    lg_stab_pos_ang.add_variable('stabilizer.yaw', 'float')
    
    lg_stab_pos_ang.add_variable('stateEstimate.x', 'float')
    lg_stab_pos_ang.add_variable('stateEstimate.y', 'float')
    lg_stab_pos_ang.add_variable('stateEstimate.z', 'float')

    ##################Speed in x, y and z#########################################

    lg_stab_vel = LogConfig(name='Stabilizer2', period_in_ms=100)
    
    lg_stab_vel.add_variable('stateEstimate.vx', 'float')
    lg_stab_vel.add_variable('stateEstimate.vy', 'float')
    lg_stab_vel.add_variable('stateEstimate.vz', 'float')


    ##################QUATERNIONS PART##############################################
    lg_stab_quat = LogConfig(name='Stabilizer3', period_in_ms=100)
    
    lg_stab_quat.add_variable('stateEstimate.qx', 'float')
    lg_stab_quat.add_variable('stateEstimate.qy', 'float')
    lg_stab_quat.add_variable('stateEstimate.qz', 'float')
    lg_stab_quat.add_variable('stateEstimate.qw', 'float')


    ##################Angle Rate#######################################################

    lg_stab_angles_rates = LogConfig(name='Stabilizer4', period_in_ms=100)
    
    lg_stab_angles_rates.add_variable('controller.r_pitch', 'float')
    lg_stab_angles_rates.add_variable('controller.r_roll', 'float')
    lg_stab_angles_rates.add_variable('controller.r_yaw', 'float')

    #Connection with the crazy fly
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        #Executes the sequence to store the logs. 
        run_sequence(scf, lg_stab_pos_ang, lg_stab_vel, lg_stab_quat, lg_stab_angles_rates)
