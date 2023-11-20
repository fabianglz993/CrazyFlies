##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 1  
# Fecha de última modificación: Semana 1
# Descripción: Código para conectar y verificar la conexión del dorn con el CrazyRadio
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Carlos Flores 


##################################################################################################################################################

#Import libraries
import logging
import time

#Import libraries to communicate with Crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper



# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E702'

#Verify connection to the drones
def simple_connect():

    print("Yeah, I'm connected! :D") # Message to indicate that the connection was succsefull
    time.sleep(3)
    print("Now I will disconnect :'(") #Message to indicate that the connection ended

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf: #It creates an instance of the drone 

        simple_connect()
