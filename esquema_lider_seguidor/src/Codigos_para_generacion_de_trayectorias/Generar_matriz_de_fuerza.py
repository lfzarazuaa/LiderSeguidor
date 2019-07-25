#!/usr/bin/env python2
# encoding: utf-8
import numpy as np
import os
import path_parser
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from scipy.spatial import KDTree

num_archivo = '3' # Ingresar del 1 al 3 o el número de trayectorias a escoger y que hayan sido previamente generadas.
puntos = 'Trayectoria'+num_archivo+'.txt' # Nombre del archivo a leer.
nombre = 'MatrizDeFuerza'+num_archivo+'.npy' # Nombre del archivo de salida que contiene la matriz de fuerza.
nombrep = 'PuntosAjustados'+num_archivo+'.npy' # Nombre del archivo de salida que contiene los puntos ajustados al mapa.
nombreGrafica1 = 'GraficoPorSeguir'+num_archivo+'.jpg' # Nombre la gráfica a seguir.
nombreGrafica2 = 'GraficoInterpolado'+num_archivo+'.jpg' # Nombre la gráfica con interpolación.
nombreGrafica3 = 'GraficoDistancias'+num_archivo+'.jpg' # Nombre la gráfica con interpolación.
map_size_x = 250.0 # Medida del mapa en "x" en cm.
map_size_y = 250.0 # Medida del mapa en "y" en cm.
resolution = 1.0 # Medida del mallado en cm.
matrix = np.zeros( (map_size_x/resolution,map_size_y/resolution,2),dtype='f' ) # Generar matriz de tamaño x*y*2 para guardar matriz de fuerza.
matrix_dist = np.zeros( (map_size_x/resolution,map_size_y/resolution),dtype='f' ) # Generar matriz de tamaño x*y*2 para guardar distancias.
ind_lookahead = np.zeros( (map_size_x/resolution,map_size_y/resolution),dtype='d' ) # Generar matriz con número de índices a avanzar. 
margen = 50 # Longitud en cm desde el borde hasta el punto mas lejano de la trayectoria.
min_dist = 0.05 # Distancia mínima de separación entre 2 puntos de trayectoria en m.

def splitall(path): # Función para separar cada directorio de una ruta.
    allparts = [] # Crea lista donde se almacenaran los datos..
    while 1:
        parts = os.path.split(path) # Usa la función split para quitar la última parte.
        print parts[0]
        print parts[1]
        if parts[0] == path:  # Sale del ciclo para rutas absolutas.
            allparts.insert(0, parts[0]) # Inserta en la posción cero de la lista la última parte que separó.
            break # Sale del ciclo
        elif parts[1] == path: # Sale del ciclo para rutas relativas.
            allparts.insert(0, parts[1]) # Inserta en la posción cero de la lista la última parte que separó.
            break # Sale del ciclo.
        else: # Guarda la ruta sin la última parte.
            path = parts[0] # La nueva ruta es la ruta sin la última parte.
            allparts.insert(0, parts[1]) # Inserta en la posción cero de la lista la última parte que separó.
    return allparts # Regresa la lista con las partes separadas.

def AjustarDatos():
    ruta = os.path.dirname(os.path.abspath(__file__))+'/Generacion_de_puntos_de_trayectoria/Archivos_de_puntos/' # Obtener la ruta del archivo.
    arr_in = np.array(list(path_parser.read_points(ruta+puntos))) # Guarda los puntos en un arreglo.
    ax,ay = arr_in.T # Separa los puntos en 2 vectores (x,y).
    min_x,min_y,max_x,max_y = minymax(ax,ay) # Calcula los mínimos y maximos de las coordenadas en "x" y en "y".
    # Imprime los valores.
    Imprimir_valores(min_x,min_y,max_x,max_y) # Imprime los valores.
    lenx,leny = [(max_x-min_x),(max_y-min_y)] # Calcula la longitud (maximo-minimo) en "x" y en "y".
    offsetx,offsety = [-(max_x-lenx/2),-(max_y-leny/2)] # Distancia para centrar en "x" y en "y".
    arr_in = arr_in+np.array([offsetx,offsety]) # Centra la trayectoria en (0,0).
    # Escalar
    ax,ay = arr_in.T
    min_x,min_y,max_x,max_y = minymax(ax,ay) # Calcula los mínimos y maximos de las coordenadas en "x" y en "y".
    Imprimir_valores(min_x,min_y,max_x,max_y) # Imprime los valores.
    lenx,leny = [(max_x-min_x),(max_y-min_y)] # Calcula la longitud (maximo-minimo) en "x" y en "y".
    scale_x,scale_y = [(map_size_x-2*margen)/(100*lenx),(map_size_y-2*margen)/(100*leny)] # Calcula los valores de escalamiento para ajustarse al mapa.
    scale = np.array([scale_x,scale_y]) # Valor de escala.
    xy = np.multiply(scale,arr_in)+np.array([(map_size_x/2)/100,(map_size_y/2)/100])#Crea los puntos de Trayectoria escalados al tamaño del mapa.
    x,y = xy.T # Obtiene los valores de "x" y "y" ya ajustados al tamaño del mapa.
    l,xo,yo = [len(x),x[0],y[0]] # Obtiene los valores iniciales de los puntos de trayectoria y la cantidad de puntos.
    # Crea una lista de puntos para ir guardando los puntos que cumplan con la distancia mínima.
    xl = [xo]
    yl = [yo]
    for i in range(l):
            xf,yf = [x[i],y[i]] # Propone como punto final el punto seleccionado por el ciclo.
            dist = np.sqrt((xf-xo)**2+(yf-yo)**2) # Calcula la distancia entre los 2 puntos.
            if dist>min_dist: # Si la distancia es mayor a la mínima la guarda el arreglo de puntos de trayectoria a seguir.
                    xo,yo = [xf,yf]# Actualiza el punto inicial.
                    # Guarda el punto en la lista.
                    xl.append(xf)
                    yl.append(yf)
    dist=np.sqrt((x[0]-xf)**2+(y[0]-yf)**2) # Calcula la distancia entre el punto inicial del arreglo y el ultimo seleccionado.
    if dist<min_dist: # Si la distancia es menor a la mínima lo elimina de la lista.
        xl.pop() # Elimina de la lista el punto en x
        yl.pop() # Elimina de la lista el punto en y
    x1,y1 = [np.array(xl),np.array(yl)] # Convierte y guarda en vectores los puntos a seguir.
    xy = np.array([x1,y1]).T# Guarda los puntos en un solo arreglo para posteriormente obtneer su árbol kd.
    xyA = np.array([x1-(map_size_x/2)/100,y1-(map_size_y/2)/100]).T # Obtiene los puntos sin el offset para guardarlos en un archivo.
    ruta = os.path.dirname(os.path.abspath(__file__))+'/Archivos_de_Puntos_Ajustados/' # Obtener la ruta para guardar los puntos.
    np.save(ruta+nombrep, xyA) # Guarda los puntos a seguir.
    print('Puntos de ruta salvados')
    fig = plt.figure(figsize=(7,7), facecolor='w') # Crea la figura para el plot.
    fig.canvas.set_window_title('Trayectoria') # Coloca el título de Trayectoria.
    # Grafica los puntos.
    plt.plot(map_size_x/2*(np.concatenate((x1,x1))-(map_size_x/200)),map_size_y/2*(np.concatenate((y1,y1))-(map_size_y/200)))
    plt.plot(map_size_x/2*(x1-(map_size_x/200)),map_size_y/2*(y1-(map_size_y/200)), ':o', markersize=4)
    plt.tight_layout() # Ajusta los titulos de subplots para evitar que salgan de la figura.
    plt.axis(np.array([-map_size_x,map_size_x,-map_size_y,map_size_y])/2)
    ruta = os.path.dirname(os.path.abspath(__file__))+'/Graficas_de_las_Trayectorias/' #Obtener la ruta para guardar las gráficas.
    plt.savefig(ruta+nombreGrafica1, bbox_inches='tight') # Guarda la imagen.
    lp = splitall(os.path.abspath(__file__))[1:-4] # Obtiene la ruta del paquete en tipo lista.
    ruta='/' # Inicializa la ruta.
    for i in range(len(lp)):
            ruta=ruta+str(lp[i])+'/' # Reconstruye la ruta en una cadena de texto.
    ruta = ruta+'gui_lider_seguidor/resources/images/ty'+num_archivo+'.png' # Obtiene la ruta total.
    plt.savefig(ruta, bbox_inches='tight') # Guarda la imagen para ser leida en la gui.
    plt.show(block=False)
    plt.show() # Muestra la gráfica.
    tree2= Arbol(xyA,30)
    return xy

def near(initial_position,xind,yind,tree,xy): # Funcion para generar matriz de fuerza.
         dist, index = tree.query(initial_position) # Obtener la distancia al punto mas cercano y el indice que lo identifica.
         global matrix_dist
         matrix_dist[xind,yind]=dist # Guardar en matriz de distancia la distancia.
         # Encontar el punto mas cercano a llegar.
         if dist>4*min_dist: # Si la distancia es mayor a 4 veces la distancia mínima de separación entre puntos
             lookahead_offset = np.int(0) # avanza con dirección al punto mas cercano.
         else: # De lo contrario
             lookahead_offset = np.int(2) # avanza con dirección al tercer punto mas cercano.
         ind_lookahead[xind,yind] = lookahead_offset # Guarda en la matriz de indices a avanzar.
         lookahead_target = xy[(index + lookahead_offset) % len(xy)] # Elige punto objetivo.
         x1, y1 = initial_position # Posición inicial.
         x3, y3 = lookahead_target # Posición a alcanzar.
         matrix[xind,yind,0] = x3-x1 # Guardar en matriz de Fuerza la distancia en x.
         matrix[xind,yind,1] = y3-y1 # Guardar en matriz de Fuerza la distancia en y.

def Arbol(xy,n): # Crea un árbol kd con interpolación y con n divisiones entre cada punto de trayectoria.
    ax,ay = xy.T # Separa "x" y "y".
    l = len(ax) # Longitud del vector.
    x = [] # Lista para la interpolación en x.
    y = [] # Lista para la interpolación en y.
    for i in range(l+1): # Itera por todos los puntos de la trayectoria.
        ind1,ind2 = [i%l,(i+1)%l] # Calcula el indice para cada punto.
        xo,xf,yo,yf = [ax[ind1],ax[ind2],ay[ind1],ay[ind2]] # Puntos iniciales y finales para realizar la interpolación lineal.
        diferenciaX,diferenciaY=[xf-xo,yf-yo] # Diferencia entre los 2 puntos.
        for j in range(n): # Agrega a la lista los n valores entre cada punto de la trayectoria.
            escala=float(j)/float(n)
            x.append(xo+diferenciaX*escala) # Calcula valores entre las 2 "x".
            y.append(yo+diferenciaY*escala) # Calcula valores entre las 2 "y".
    x1,y1=[np.array(x),np.array(y)] # Convierte los valores a vectores.
    xy=np.array([x1,y1]).T # Guarda en una matriz los datos.
    # Grafica la trayectoria con una interpolación lineal.
    fig2 = plt.figure(figsize=(7,7), facecolor='w')
    fig2.canvas.set_window_title('Interpolacion')
    plt.plot(x1,y1,'*b')
    plt.axis([-1,1,-1,1])
    ruta = os.path.dirname(os.path.abspath(__file__))+'/Graficas_de_las_Trayectorias/' #Obtener la ruta para guardar las gráficas.
    plt.savefig(ruta+nombreGrafica2, bbox_inches='tight') # Guarda la gráfica interpolada.
    plt.show(block=False)
    plt.show()
    return KDTree(xy) # Regresa el arbol kd ya interpolado.

def minymax(ax,ay): # Devuelve maximos y mínimos de 2 arreglos.
    return np.array([np.min(ax),np.min(ay),np.max(ax),np.max(ay)])

def Imprimir_valores(min_x,min_y,max_x,max_y):
    print 'Minimo en x',min_x
    print 'Minimo en y',min_y
    print 'Maximo en x',max_x
    print 'Maximo en y',max_y

def GenerarMatriz(xy): # Genera la matriz con los puntos ya escalados.
    tree = KDTree(xy)
    print('Espere por favor ...')
    # Crear mallado
    X = np.arange(0,map_size_x/100,resolution/100)
    Y = np.arange(0,map_size_y/100,resolution/100)
    X,Y = np.meshgrid(X,Y)
    # Definir límites del mapa
    lim_x = int(map_size_x/resolution)
    lim_y = int(map_size_y/resolution)
    # Calcular matriz de fuerza para cada punto del mallado
    for xi in range(0, lim_x):
        print float(xi)/lim_x*100
        for yi in range(0, lim_y):
            near((xi*resolution/100,yi*resolution/100),xi,yi,tree,xy)
    Z = matrix_dist#Guarda en Z la matriz de distancias.
    # Grafica el espacio de distancias.
    fig = plt.figure(figsize=(7,7), facecolor='w')
    eje = fig.gca(projection='3d')
    fig.canvas.set_window_title('Distancias')
    surf = eje.plot_surface(X, Y, Z, cmap=cm.coolwarm,linewidth=0, antialiased=False)
    ruta = os.path.dirname(os.path.abspath(__file__))+'/Graficas_de_las_Trayectorias/' # Obtener la ruta para guardar las gráficas.
    plt.savefig(ruta+nombreGrafica3, bbox_inches='tight') # Guarda la figura con la imagen.
    plt.show(block=False)
    plt.show()
    ruta = os.path.dirname(os.path.abspath(__file__))+'/Archivos_de_Trayectoria/' # Obtener la ruta para guardar los puntos.
    np.save(ruta+nombre, matrix) # Guarda los puntos a seguir.
    print('Matriz de fuerza salvada')

def main(): # Función Principal
    xy=AjustarDatos() # Ajusta los datos al tamaño del mapa.
    GenerarMatriz(xy) # Genera la matriz de Fuerza.

if __name__ == '__main__':
    main()
    cadena='Programa Finalizado'