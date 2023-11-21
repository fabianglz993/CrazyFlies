##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 1  
# Fecha de última modificación: Semana 9
# Descripción: Gráfica el movimiento de dos drones para análisis de datos
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: David Padilla

##################################################################################################################################################
#Import libraries
from matplotlib import pyplot as plt
import pandas as pd

def graph(df, df1):
    # Function to plot various graphs based on DataFrame 'df'
    ############## 3D Graph ##################

    plt.figure('3D trajectory mapping') # It creates a new figure with name 3d trayectory mapping
    ax = plt.axes(projection ="3d") # Gnerates 3 axes to graph in 3D
    plot = ax.scatter3D(df['stateEstimate.x'],df['stateEstimate.y'],df['stateEstimate.z'], c = df['time']/10000 ,cmap = 'plasma') # The positions in X,Y,Z of drone 1 are graphed with respect to time, time is represented by a heat map
    plot1 = ax.scatter3D(df1['stateEstimate.x'],df1['stateEstimate.y'],df1['stateEstimate.z'], c =df1['time']/10000, cmap = 'plasma') # The positions in X,Y,Z of drone 2 are plotted with respect to time, time is represented by a heat map
    ax.set_xlabel('position in X [m]') # The x axis is named position in X [m]
    ax.set_ylabel('Position in Y [m]') # The y axis is named position in Y [m]
    ax.set_zlabel('Position in Z [m]') # The z axis is named position in Z [m]
    plt.colorbar(plot,cmap=plt.cm.magma,label = 'Time [s]') # The color bar is included in order to interpret the color map, the scale is in seconds
    plt.title('3D trajectory mapping') # A title is added to the graph 

    ############## XY Plane ##################

    plt.figure('XY Plane') # Create a new shape with the name XY Plane
    plot = plt.scatter(df['stateEstimate.x'], df['stateEstimate.y'], c = df['time']/10000, cmap='plasma') #The positions in X,Y of drone 1 are plotted with respect to time, the time is represented by a heat map
    plot1 = plt.scatter(df1['stateEstimate.x'] ,df1['stateEstimate.y'],c = df1['time']/10000, cmap='plasma') # The positions in X,Y of drone 2 are plotted with respect to time, time is represented by a heat map
    plt.colorbar(plot, cmap=plt.cm.magma, label = 'Time [s]') #The color bar is included in order to interpret the color map, the scale is in seconds
    # circle1 = plt.Circle((2, 4), 0.2, color='black', fill = False, label = 'Error zone')          # Comment or uncomment to graph a circle with center in XY coordinates of radius 0.2
    # circle2 = plt.Circle((1.47, 3.15), 0.2, color='black', fill = False, label = 'Error zone')    # Comment or uncomment to graph a circle with center in XY coordinates of radius 0.2
    # plt.gca().add_patch(circle1)
    # plt.gca().add_patch(circle2)   
    plt.xlabel('Position in X [m]') # The x axis is named position in X [m]
    plt.ylabel('Position in Y [m]') # The y axis is named position in Y [m]
    plt.title('XY Plane') #  A title is added to the graph  
    plt.grid() # A grid is added to the bottom of the graph

    ############## Height vs Time ##################

    plt.figure('Height vs Time') # Create a new shape named Height vs Time
    plt.scatter(df['time']/10000, df['stateEstimate.z'], label = 'Measures height drone 1') # The height position of drone 1 is plotted against time
    plt.scatter(df1['time']/10000, df1['stateEstimate.z'], label = 'Real height drone 2', color = 'green') # The position of the height of drone 2 is plotted against time
    plt.grid() # A grid is added to the bottom of the graph
    plt.axhline(1.2, color ='red', label = 'Max') # Graph a horizontal line to represent the upper limit of the proposed requirements.
    plt.axhline(0.8, color = 'black', label = 'Min') # Graph a horizontal line to represent the lower limit of the proposed requirements.
    plt.xlabel('Time [s]') #The x axis is named Time [s]
    plt.ylabel('Height [m]') # The y axis is named Height [m]
    plt.legend() # The legend is added to identify the data in the graph
    plt.title('Height vs Time') # Add a title to the graph
    plt.show() # The generated graphs are displayed

file = r'D:\Downloads\01_06_23_Prueba027_3_LiderCon2Drones_drone1.csv'
file1 = r'D:\Downloads\01_06_23_Prueba027_3_LiderCon2Drones_drone2.csv'
df = pd.read_csv(file, index_col= 0)
df1 = pd.read_csv(file1, index_col= 0)

graph(df,df1)
