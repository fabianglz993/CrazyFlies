##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 1  
# Fecha de última modificación: Semana 9
# Descripción: Obtiene el valor máximo y minimo de algunos párametros para fines de verificación y validación
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: David Padilla

##################################################################################################################################################
#Import libraries
import os
import pandas as pd

path = "D:\Downloads\Logs_pruebas" # Creates the path to open the folder that we will later define as address
dir = os.listdir( path ) # Opens the address defined in path
df1 = pd.DataFrame() # Create an empty dataframe

for file in dir: # Iterates over the different files saved within the dir folder
    df = pd.read_csv(path + r'\\' + file, index_col = 0)  # The different .csv files are read and converted to a Pandas Dataframe 
    df1 =df1.append(df,ignore_index = True) # The dataframes are added in each iteration to the dataframe that we defined before the for loop

print('La velocidad máxima en x es:', df1['stateEstimate.vx'].max()) # Displays in terminal the maximum value of the speed in the X axis [m/s]
print('La velocidad máxima en y es:', df1['stateEstimate.vy'].max()) # Displays in terminal the maximum value of the speed in the Y axis [m/s]
print('La velocidad máxima en z es:', df1['stateEstimate.vz'].max()) # Displays in terminal the maximum value of the speed in the Z axis [m/s]
print('El ángulo máximo en yaw es:', df1['stabilizer.yaw'].max()) # Displays in terminal the maximum value of the angle in yaw [°]
print('El ángulo minimo en yaw es:', df1['stabilizer.yaw'].min()) # Displays in terminal the minimum value of the angle in yaw [°]

