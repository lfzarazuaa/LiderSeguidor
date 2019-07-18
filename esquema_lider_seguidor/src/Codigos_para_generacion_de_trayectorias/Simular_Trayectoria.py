#!/usr/bin/env python2
# encoding: utf-8
import numpy as np
import path_parser
import os
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from scipy.spatial import KDTree

map_size_x=250.0 # Medida del mapa en "x" en cm.
map_size_y=250.0 # Medida del mapa en "y" en cm.
resolution = 1.0 # Medida del mallado en cm.
num_archivo='1'#Ingresar del 1 al 3 o el número de trayectorias a escoger y que hayan sido previamente generadas.
nombre='MatrizDeFuerza'+num_archivo+'.npy'# Nombre del archivo a leer que contiene la matriz de fuerza.

def main():
    ruta=os.path.dirname(os.path.abspath(__file__))+'/Archivos_de_Trayectoria/' #Obtener la ruta para leer la matriz de fuerza.
    matrix = np.load(ruta+nombre)
    final,div,x1,y1=[400,4,0,0]#Variables para simular la llegada a un punto del mapa.
    x,y=[np.zeros(final,dtype='f'),np.zeros(final,dtype='f')]#Inicializar los puntos de simulación.
    for xi in range(0, final):
        initial_position=[x1,y1]#Comenzar en la posicion inicial indicada.
        pos_x,pos_y=[x1+map_size_x/200,y1+map_size_y/200]#Obtiene la posición con el mapa desplazado.
        x_index,y_index=[np.int(pos_x*(100/resolution)),np.int(pos_y*(100/resolution))]#Obtiene el posible indice de la posición asociada.
        if (x_index<0):#Define el límite de indice en x.
            x_index = 0
        elif (x_index>((map_size_x/resolution)-1)):
            x_index=(map_size_x/resolution)-1
        if (y_index<0):#Define el límite de indice en y.
            y_index = 0
        elif (y_index>((map_size_y/resolution)-1)):
            y_index=(map_size_y/resolution)-1
        x2, y2 = matrix[x_index,y_index,:]#Lee la posición a llegar a partir de ingresar los indices a la matriz de fuerza.
        x[xi],y[xi]=[float(x1),float(y1)]#Guarda la posición en el arreglo de posiciones alcanzadas.
        x1,y1=[x1+x2/div,y1+y2/div]#Hace de la posición inicial la posición a llegar dividido entre n que simula la muestra discreta de posición que toma el vehículo.
    plt.plot(x,y)#Grafica el resultado de las posiciones alcanzadas.
    plt.title('Trayectoria Simulada [m]')
    plt.show()

if __name__ == '__main__':
    main()
