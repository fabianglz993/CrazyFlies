# CrazyFlies

Sistema de vuelo colaborativo entre varios drones

## Instalación / Prerequisitos
#Instalación de cfcclien

cfcclient es una interfaz gráfica para observar el sistema de posicionamiento.

1. Revisar que se cuente con la versión de python3 en la terminal de Powershell

```
python --version
pip --version
pip3 install --upgrade pip

```
2. Instalar [Zadig](http://zadig.akeo.ie/.) para descargar los drivers (se utilizará para el dron y el crazyradio).

3. Conectar el Crazyradio en la computadora y abrir zadig
4. Seleccionar "crazyradio" en los devices.
5. Seleccionar "libusb-win32" y seleccionar instalar drivers.
6. Una vez que se han instalado correctamente habrás terminado.
7. Para comprobar la instalación corre el siguiente comando en una terminal

```
pip3 install cfclient

```
8. Una vez instalada corre la interfaz con el siguiente comando 
```
python3 -m cfclient.gui 

```
también se puede utilizar el siguiente comando

```
cfclient 
```

## Introducción / Instrucciones

La carpeta "Proyectos Base" contiene programas que son esenciales para conocer las funciones basicas de este proyecto, desde la conexión a un dron, hasta el control de posición de dos drones. 

La carpeta "Simulación y Analisis" contiene programas que ofrecen un apoyo visual como gráficas y simulaciones que sirven de apoyo para diseñar el proyecto.

## Features

## Recursos

## Autores




