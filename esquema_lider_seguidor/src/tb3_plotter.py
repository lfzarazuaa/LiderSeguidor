#!/usr/bin/env python
# encoding: utf-8
import sys # Importar libería para recibir variables del sistema o consola.
import rospy # Importar libería de ROS para python.
import numpy as np # Importar libería matemática.
import os # Importar librería para leer carpetas.
import time # Importar libería para obtener el tiempo apartir del sistema.
import matplotlib.pyplot as plt # Importar líbreria para gráficar.
import pandas as pd # Importa librería para manipular archivos de excel.
from scipy.spatial import KDTree # Importa librería para generar árboles KD.
from tf.transformations import euler_from_quaternion, quaternion_from_euler # Importa librería para hacer transformaciones en la orientación.
# Importa mensajes de ROS
from geometry_msgs.msg import PoseWithCovarianceStamped
from turtlesim.msg import Pose
from std_msgs.msg import Float64,Int32,Bool

num_trayectorias =3 # Define cuantas trayectorias puede seguir.
p = 20 # Número de puntos del árbol entre 2 posiciones consecutivas en la trayectoria.
map_size_x = 250.0 # Medida del mapa en "x" en cm.
map_size_y = 250.0 # Medida del mapa en "y" en cm.

class Plotter: # Clase para graficar los puntos alcanzados.
    def __init__(self): # Define el constructor de la clase.
        self.poseflag = False # Define bandera de posición en falso.
        ruta = os.path.dirname(os.path.abspath(__file__))+'/Codigos_para_generacion_de_trayectorias/Archivos_de_Puntos_Ajustados/' # Ruta para guardar los datos obtenidos.
        self.Arboles = [] # Define lista donde se guardaran los árboles KD de cada trayectoria.
        for i in range(num_trayectorias): # Ciclo para guardar los distintos árboles KD.
            self.Arboles.append(self.Arbol(np.array(np.load(ruta+'PuntosAjustados'+str(i+1)+'.npy')),p)) # Guarda el árbol KD correspondiente.
        self.turtlebot3_pose_L = Pose() # Crea objeto para almacenar la posición del líder.
        self.Positions_XL = [] # Crea objeto para almacenar la posiciónes en "x" del líder.
        self.Positions_YL = [] # Crea objeto para almacenar la posiciónes en "y" del líder.
        self.Positions_EL = [] # Crea objeto para almacenar el error de distancia del líder.
        self.turtlebot3_pose_F = Pose() # Crea objeto para almacenar la posición del líder.
        self.Positions_XF = [] # Crea objeto para almacenar la posiciónes en "x" del seguidor.
        self.Positions_YF = [] # Crea objeto para almacenar la posiciónes en "y" del seguidor.
        self.Positions_EF = [] # Crea objeto para almacenar el error de distancia del seguidor.
        self.Positions_Count = [] # Crea objeto para almacenar cuantos puntos alcanzados se tienen.
        self.count = 0 # Declara el contador de puntos alcanzados en cero.
        self.lane = 0 # Declara sin trayectoria seleccionada.
        self.graph = False # Declara la bandera de graficar en falso.
        self.clear = False # Declara la bandera de limpiar gráfica en falso.
        self.save_data = False # Declara la bandera de guardar datos en falso.
        self.language = False # Declara el idioma en español.
        self.rows = 2 # Número de filas en el graficador.
        self.columns = 1 # Número de columnas en el graficador.
        self.map_size_x = float(map_size_x) # Medida del mapa en "x" en cm.
        self.map_size_y = float(map_size_y) # Medida del mapa en "y" en cm.
        # Declara los distintos subscriptores.
        self.lane_subscriber = rospy.Subscriber("/lane",Int32,self.laneCallback,queue_size=1) # Asocia el método cuando se recibe que trayectoria se eligió.
        self.posel_subscriber = rospy.Subscriber("/tb3_0/amcl_pose", PoseWithCovarianceStamped,self.poseCallback,queue_size=1) # Asocia el método que indica que hacer cuando la posición del líder es recibida.
        self.posef_subscriber = rospy.Subscriber("/tb3_1/amcl_pose", PoseWithCovarianceStamped,self.poseCallback2,queue_size=1) # Asocia el método que indica que hacer cuando la posición del seguidor es recibida.
        self.graph_subscriber = rospy.Subscriber("/graph", Bool,self.graph_on_Callback,queue_size=1) # Asocia el método para el bit de graficar o no.
        self.clear_graph_subscriber = rospy.Subscriber("/clear_graph",Bool,self.clear_graph_Callback,queue_size=1) # Asocia el método para el evento de limpiar gráfica.
        self.save_data_subscriber = rospy.Subscriber("/save_data",Bool,self.save_data_Callback,queue_size=1) # Asocia el método para el evento de guardar datos.
        self.change_language_subscriber = rospy.Subscriber("/change_language",Bool,self.change_language_Callback,queue_size=1) # Asocia el método para el bit del idioma.
        plt.ion() # Activa modo interactivo.

    def poseCallback(self,data): # Método manejador de la posición del líder.
        self.turtlebot3_pose_L.x, self.turtlebot3_pose_L.y=[data.pose.pose.position.x,data.pose.pose.position.y] # Guarda la posición del líder.
        self.poseflag = True # Activa bandera de posición recibida.
    
    def poseCallback2(self,data): # Método manejador de la posición del seguidor.
        self.turtlebot3_pose_F.x,self.turtlebot3_pose_F.y=[data.pose.pose.position.x, data.pose.pose.position.y] # Guarda la posición del seguidor.
        self.poseflag = True # Activa bandera de posición recibida.

    def laneCallback(self,data): # Método manejador de la función para recibir la trayectoria seleccionada.
        self.lane = data.data # Guarda la trayectoria seleccionada.

    def graph_on_Callback(self,data): # Método manejador para activar graficación de los datos.
        self.graph = data.data # Guarda si se quiere graficar o no.

    def clear_graph_Callback(self,data): # Método manejador de la opción para limpiar grafica.
        self.clear = True # Pone en alto la bandera para limpiar la gráfica.

    def change_language_Callback(self,data): # Método manejador de la opción para cambiar de idioma.
        self.language = data.data # Guarda el idioma seleccionado.
        
    def save_data_Callback(self,data): # Método manejador de la opción para guardar datos de puntos alcanzados.
        if self.count>2: # Si se tienen mas de 2 puntos alcanzados.
            self.save_data = True # Pone en alto la bandera para guardar los puntos alcanzados.

    def mostrar_graficas(self): # Método graficar los puntos alcanzados y ejecutar otras opciones de usuario.
        self.rate = rospy.Rate(20) # Define el tiempo de espera de 20 ms.
        if self.save_data==True: # Si se activó la bandera de guardar puntos alcanzados.
            self.save_data = False # Descativa la bandera para evitar guardar 2 veces.
            # Convierte a arreglos las listas con los puntos en "x", "y" y el error.
            po=np.array(self.Positions_Count)
            xl=np.array(self.Positions_XL)
            yl=np.array(self.Positions_YL)
            el=np.array(self.Positions_EL)
            xf=np.array(self.Positions_XF)
            yf=np.array(self.Positions_YF)
            ef=np.array(self.Positions_EF)
            # Guarda en una matriz los datos obtenidos.
            matrix = np.array([po,xl,yl,el,xf,yf,ef]).T
            ruta=os.path.dirname(os.path.abspath(__file__)) # Obtiene la ruta para guardar los datos.
            nombre_Archivo ='/Datos_de puntos_alcanzados/Grafica_'+time.strftime("%Y_%m_%d_%H_%M_%S") # Asigna un nombre al archivo con su fecha y hora.
            nombre_ArchivoP = nombre_Archivo+'.npy' # Extensión para leer en python.
            nombre_ArchivoE = nombre_Archivo+'.xlsx' # Extensión para leer datos en excel.
            np.save(ruta+nombre_ArchivoP, matrix) # Guarda el archivo para leerse en python.
            df = pd.DataFrame(matrix,columns = ['Muestra','x Lider','y Lider','error Lider','x Seguidor','y Seguidor','error Seguidor']) # Da nombre de las columnas en excel.
            df.to_excel(ruta+nombre_ArchivoE, sheet_name ='Datos Obtenidos') # Guarda los datos obtenidos.
            print 'Datos guardados' # Imprime en pantalla que los datos fueron guardados.
        
        if self.clear==True:  # Si se activó la bandera de guardar puntos alcanzados.
            self.clear = False # Descativa la bandera para evitar que se limpie en cada momento.
            self.count = 0 # Reinicia el contador de posiciones alcanzadas.
            # Reinicia los listas con los puntos en "x", "y" y el error.
            self.Positions_Count = []
            self.Positions_XL = []
            self.Positions_YL = []
            self.Positions_EL = []
            self.Positions_XF = []
            self.Positions_YF = []
            self.Positions_EF = []
            # Limpia las gráficas y sus textos.
            plt.subplot(self.rows,self.columns,1) # Elige la gráfica 1.
            plt.cla() # Limpia la gráfica 1.
            if self.language==True: # Si el idioma es inglés.
                plt.title('Reached Positions (Lider-Red,Follower-Blue)[m]') # Coloca el título en inglés.
            else: # Si es español.
                plt.title('Posiciones Alcanzadas (Lider-Rojo,Seguidor-Azul)[m]') # Coloca el título en español.
            plt.draw() # Dibuja la gráfica.
            plt.pause(0.00000000001) # Espera un tiempo para ejecutar correctamente el comando.
            plt.subplot(self.rows,self.columns,2) # Elige la gráfica 2.
            plt.cla() # Limpia la gráfica 1.
            if self.language==True:  # Si el idioma es inglés.
                plt.title('Error Graph [m]') # Coloca el título en inglés.
            else: # Si es español.
                plt.title('Error Graficado [m]') # Coloca el título en español.
            plt.draw() # Dibuja la gráfica.
            plt.pause(0.00000000001) # Espera un tiempo para ejecutar correctamente el comando.
            print 'Grafica Limpiada' # Imprime en pantalla que se limpió la gráfica.

        if self.graph==True: # Si se eligio la opción de graficar.
            plt.subplot(self.rows,self.columns,1) # Elige la gráfica 1.
            plt.hold(True) # No borra la gráfica anterior al momento de gráficar la nueva.            
            plt.plot(self.turtlebot3_pose_L.x,self.turtlebot3_pose_L.y,'*r') # Grafica la posición del líder.
            plt.plot(self.turtlebot3_pose_F.x,self.turtlebot3_pose_F.y,'*b') # Grafica la posición del seguidor.
            plt.axis([-self.map_size_x/200,self.map_size_x/200,-self.map_size_y/200,self.map_size_y/200]) # Fija los límites de la gráfica.
            if self.language==True: # Si el idioma es inglés.
                plt.title('Reached Positions (Lider-Red,Follower-Blue)[m]') # Coloca el título en inglés.
            else:
                plt.title('Posiciones Alcanzadas (Lider-Rojo,Seguidor-Azul)[m]') # Coloca el título en español.
            plt.draw() # Dibuja la gráfica.
            plt.pause(0.00000000001) # Espera un tiempo para ejecutar correctamente el comando.
        
            if self.lane>0: # Si se seleccionó una trayectoria válida.
                # Almacenar posiciones en una lista para cuando se requieran guardar.
                self.Positions_XL.append(self.turtlebot3_pose_L.x) # Almacena posición del líder en x.
                self.Positions_YL.append(self.turtlebot3_pose_L.y) # Almacena posición del líder en y.
                self.Positions_XF.append(self.turtlebot3_pose_F.x) # Almacena posición del seguidor en x.
                self.Positions_YF.append(self.turtlebot3_pose_F.y) # Almacena posición del seguidor en y.
                plt.subplot(self.rows,self.columns,2)  # Elige la gráfica 1.
                plt.hold(True) # No borra la gráfica anterior al momento de gráficar la nueva.
                distL, index = self.Arboles[self.lane-1].query((self.turtlebot3_pose_L.x,self.turtlebot3_pose_L.y)) # Obtiene el error de distancia en el líder.
                distF, index = self.Arboles[self.lane-1].query((self.turtlebot3_pose_F.x,self.turtlebot3_pose_F.y)) # Obtiene el error de distancia en el seguidor.
                self.Positions_EL.append(distL) # Almacena el error del líder.
                self.Positions_EF.append(distF) # Almacena el error del seguidor.
                self.Positions_Count.append(self.count) # Almacena el contador de puntos alcanzados.
                plt.plot(self.count,distL,'*r') # Gráfica el error del líder.
                plt.plot(self.count,distF,'*b') # Gráfica el error del seguidor.
                self.count=self.count+1 # Incrementa el contador de puntos alcanzados.
                plt.axis([0,self.count,0,0.8]) # Define los límites de la gráfica del error.
                if self.language==True: # Si el idioma es inglés
                    plt.title('Error Graph ('+ str(distL) + ',' + str(distF) + ')[m]') # Coloca el título en inglés.
                else: # Si el idioma es español
                    plt.title('Error Graficado('+ str(distL) + ',' + str(distF) + ')[m]') # Coloca el título en español.
                plt.draw() # Dibuja la gráfica.
                plt.pause(0.00000000001) # Espera un tiempo para ejecutar correctamente el comando.
            else: # Si no hay trayectoria seleccionada.
                plt.subplot(self.rows,self.columns,2) # Selecciona la gráfica 2.
                if self.language==True: # Si el idioma es inglés.
                    plt.title('Error Graph [m]') # Coloca el título en inglés.
                else: # Si el idioma es español
                    plt.title('Error Graficado [m]') # Coloca el título en español.
                plt.pause(0.00000000001) # Espera un tiempo para ejecutar correctamente el comando.
        self.rate.sleep() # Espera el tiempo indicado.

    def Arbol(self,xy,longitud): # Método para crear un árbol KD apartir de los puntos.
        ax,ay = xy.T # Descompone el arreglo en 2 vectores.
        l = len(ax) # Calcula cuantos puntos tiene el vector.
        x = [] # Crea una lista de "x".
        y = [] # Crea una lista de "y".
        for i in range(l+1): # Realiza un escaneo por cada elemento.
            ind1,ind2 = [i%l,(i+1)%l] # Calcula el indice para cada punto.
            xo,xf,yo,yf = [ax[ind1],ax[ind2],ay[ind1],ay[ind2]] # Puntos iniciales y finales para realizar la interpolación lineal.
            diferenciaX,diferenciaY = [xf-xo,yf-yo] # Diferencia entre los 2 puntos.
            for j in range(longitud): # Agrega a la lista los n valores entre cada punto de la trayectoria.
                escala = float(j)/float(longitud)
                x.append(xo+diferenciaX*escala) # Calcula valores entre las 2 "x".
                y.append(yo+diferenciaY*escala) # Calcula valores entre las 2 "y".
        x1,y1 = [np.array(x),np.array(y)] # Convierte los valores a vectores.
        xy = np.array([x1,y1]).T # Guarda en una matriz los datos.
        return KDTree(xy) # Regresa el arbol KD de los puntos ingresados.         

def main(): # Función principal
        rospy.init_node('Plotter', anonymous=True) # Inicia el nodo del graficador.
        P = Plotter() # Constructor de la clase Plotter.
        print 'PLotter inicializado' # Imprime mensaje de función principal inciada.
        while (not rospy.is_shutdown()):
            P.mostrar_graficas() # Ir graficando los puntos.
        #rospy.spin()
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
