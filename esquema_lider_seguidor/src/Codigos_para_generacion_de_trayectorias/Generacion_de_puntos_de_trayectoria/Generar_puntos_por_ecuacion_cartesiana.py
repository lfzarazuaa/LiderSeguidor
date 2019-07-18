#!/usr/bin/env python2
# encoding: utf-8
import numpy as np
import path_parser
import matplotlib.pyplot as plt
import os
nombre_de_archivo_de_salida='Trayectoria2'#Nombre del Archivo generado.

def GuardarArchivo(nombre):
    xr,yr=puntos#Puntos a guardar.
    npuntos=len(xr)#Número de puntos.
    ruta=os.path.dirname(os.path.abspath(__file__))#Ruta donde se ejecuta el programa.
    archivo=open(ruta+'/Archivos_de_puntos/'+nombre+'.txt','w')#Empezar a escribir en el archivo de texto.
    #Definicion del archivo de coordenadas o puntos.
    archivo.write('/* Sample RNDF change log*/\n')
    archivo.write('/*                                                 */\n')
    archivo.write('/* Mar 21, 2019 - RNDF - Anticlockwise road selection*/\n')
    archivo.write('/*                                                 */\n')
    archivo.write('/*                                                 */\n')    
    archivo.write('RNDF_name RNDF_Rev_1.0\n')
    archivo.write('num_segments 1\n')
    archivo.write('num_zones 0\n')
    archivo.write('format_version 1.0\n')
    archivo.write('creation_date 21-Mar-19\n')
    archivo.write('segment 1\n')
    archivo.write('num_lanes 1\n')
    archivo.write('segment_name Outer_Loop\n')
    archivo.write('lane 1.2\n')
    archivo.write('num_waypoints '+ str(npuntos) + '\n')
    archivo.write('lane_width 0.35\n')
    archivo.write('left_boundary solid_white\n')
    archivo.write('right_boundary broken_white\n')
    #Imprime cada uno de los puntos en el documento.
    for i in range(1,npuntos+1):
        archivo.write('1.2.' + str(i) + '	' + str(xr[i-1]) +'	'+ str(yr[i-1]) + '\n')
    archivo.close()#Termina de editar el archivo.

def cvector(inicio,final,paso):#Función para generar vector que su final el número indicado.
    return np.arange(inicio,final+paso,paso)
    
def main():
    m,n=(1,2);
    A=0.3;
    f=1;# 3Hz
    x=np.arange(0.25,2.26,0.01);
    w=2*np.pi*f;
    #Definición de funcion matemática.
    y1=-A*np.sin(w*(x-0.25))+1
    y3=A*np.sin(w*(x-0.25))+2
    final=len(x)-1
    y4=cvector(y3[0],y1[0],-0.01)
    y2=cvector(y1[final],y3[final-1],0.01)
    plt.ioff()#Desactivar modo interactivo
    x2=x[final]*(y2*0+1)
    x4=x[0]*(y4*0+1)
    x3=x[::-1];
    y3=y3[::-1];
    #Imprimir funciones a trozos.
    plt.subplot(m,n,1)
    plt.title('Puntos de Trayectoria')
    plt.plot(x,y1)
    plt.plot(x2,y2)
    plt.plot(x3,y3)
    plt.plot(x4,y4)
    plt.subplot(m,n,1)
    #Concatenar todos los datos en una sola función.
    xr=np.concatenate((x,x2,x3,x4));
    yr=np.concatenate((y1,y2,y3,y4));
    plt.subplot(m,n,2)
    plt.title('Trayectoria cerrada')
    plt.scatter(xr,yr)
    plt.show()
    global puntos
    puntos=(xr,yr)
    #Guardar archivo con puntos.
    GuardarArchivo(nombre_de_archivo_de_salida);

if __name__ == '__main__':
    main()