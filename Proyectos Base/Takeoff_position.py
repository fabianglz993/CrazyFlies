##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 3  
# Fecha de última modificación: Semana 3
# Descripción: 
#   Código para realizar un despegue del dron a 1 metro de altura y mantenerla durante 15s con el sistema CrazyRadio 
#   sin movimientos en (x,y)
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Misael Cruz

##################################################################################################################################################
#  Copyright (C) 2019 Bitcraze AB


'''
Librerias
'''
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

'''
Variables globales
'''
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702') # URI to the Crazyflie to connect to
sequence = [(0, 0, 1),     
            (0, 0, 1),     
            (0, 0, 1),     
            (0, 0, 0.2),
           ] #Secuencia para mantener la altura a 1 metro dependiendo de dónde inicie el dron, descritos en formato (x,y,z)
             #La secuencia (0, 0, 1) Mantiene la altura mientras que la secuencia (0,0,0.2) baja a una zona segura

''' 
Funciones
'''
def run_sequence(scf, _sequence, _initial_x, _initial_y, _initial_z, yaw):
    """Manda a ejecutar el código donde se mantendrá la altura del dron, se modifica cada posicion en sequence para que sea absoluta al sistema Loco
        Args:
            scf ():  Instancia sincrónica de crazyfile
            sequence (): Secuencia de posiciones creada para mantener la altura
            _initial_x (float): Posicion inicial en eje X del dron en metros
            _initial_y (float): Posicion inicial en eje Y del dron en metros
            _initial_z (float): Posicion inicial en eje Z del dron en metros
    """
    cf = scf.cf
    for position in _sequence:
        print('Setting position {}'.format(position))
        x = position[0] + _initial_x
        y = position[1] + _initial_y
        z = position[2] + _initial_z
        for i in range(50):
            cf.commander.send_position_setpoint(x, y, z, yaw) #Manda el comando con la posición (x,y,z) y ángulo (yaw) que debe tener el dron
            time.sleep(0.1)
    cf.commander.send_stop_setpoint()
    time.sleep(0.1)

'''
Main
'''
if __name__ == '__main__':
    cflib.crtp.init_drivers()
    
    # Valores de posición inicial dependiendo de dónde se coloca el dron en metros
    initial_x = 1.5
    initial_y = 3.15
    initial_z = 0.0   
    initial_yaw = 90  # En grados

#Se inicia la comunicación con el dron y se ejecuta la función de posicionamiento
with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    run_sequence(scf, sequence, initial_x, initial_y, initial_z, initial_yaw)