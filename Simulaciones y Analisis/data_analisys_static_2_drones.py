##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 1  
# Fecha de última modificación: Semana 9
# Descripción: Gráfica la altura de dos drones para análisis de datos
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: David Padilla

##################################################################################################################################################
#Import libraries
from matplotlib import pyplot as plt
import pandas as pd

def graph(df, df1):
    # Function to plot height versus time for two drones

    ############## Height vs Time ##################

    # Create a scatter plot of height against time for drone 1
    plt.figure('Height vs Time')
    plt.scatter(df['time']/10000, df['stateEstimate.z'], label='Measures height drone 1')

    # Create a scatter plot of height against time for drone 2 in green
    plt.scatter(df1['time']/10000, df1['stateEstimate.z'], label='Real height drone 2', color='green')

    plt.grid()
    plt.axhline(1.2, color='red', label='Max')
    plt.axhline(0.8, color='black', label='Min')
    plt.xlabel('Time [s]')
    plt.ylabel('Height [m]')
    plt.legend()
    plt.title('Height vs Time')
    plt.show()

# Read data from CSV files into DataFrames
file = r'D:\Downloads\logs_pruebas1dron_31Mayo\31_05_23_Prueba025_3_VueloEstaticoDosDrones_drone1.csv'
file1 = r'D:\Downloads\logs_pruebas1dron_31Mayo\31_05_23_Prueba025_3_VueloEstaticoDosDrones_drone2.csv'
df = pd.read_csv(file, index_col=0)
df1 = pd.read_csv(file1, index_col=0)

# Call the graph function to visualize the height versus time for two drones
graph(df, df1)
