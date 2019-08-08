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

num_trayectorias=3 # Define cuantas trayectorias puede seguir.
m=2 # Número de filas en el graficador.
n=1 # Número de columnas en el graficador.
p=20 # Número de puntos del árbol entre 2 posiciones consecutivas en la trayectoria.

class Plotter: # Clase para graficar los puntos alcanzados.
    def __init__(self): # Define el constructor de la clase.
      self.poseflag = False # Define bandera de posición en falso.
      ruta = os.path.dirname(os.path.abspath(__file__))+'/Codigos_para_generacion_de_trayectorias/Archivos_de_Puntos_Ajustados/' # Ruta para guardar los datos obtenidos.
      self.Arboles = [] # Define lista donde se guardaran los árboles KD de cada trayectoria.
      for i in range(num_trayectorias): # Ciclo para guardar los distintos árboles KD.
        self.Arboles.append(self.Arbol(np.array(np.load(ruta+'PuntosAjustados'+str(i+1)+'.npy')),p)) # Guarda el árbol KD correspondiente.
      self.turtlebot3_pose_L = Pose() # Crea objeto para almacenar la posición del líder.
      self.Positions_XL=[] # Crea objeto para almacenar la posiciónes en "x" del líder.
      self.Positions_YL=[] # Crea objeto para almacenar la posiciónes en "y" del líder.
      self.Positions_EL=[] # Crea objeto para almacenar el error de distancia del líder.
      self.turtlebot3_pose_F = Pose() # Crea objeto para almacenar la posición del líder.
      self.Positions_XF=[] # Crea objeto para almacenar la posiciónes en "x" del seguidor.
      self.Positions_YF=[] # Crea objeto para almacenar la posiciónes en "y" del seguidor.
      self.Positions_EF=[] # Crea objeto para almacenar el error de distancia del seguidor.
      self.Positions_Count=[] # Crea objeto para almacenar cuantos puntos alcanzados se tienen.
      self.count=0 # Declara el contador de puntos alcanzados en cero.
      self.lane=0 # Declara sin trayectoria seleccionada.
      self.graph=False # Declara la bandera de graficar en falso.
      self.clear=False # Declara la bandera de limpiar gráfica en falso.
      self.save_data=False # Declara la bandera de guardar datos en falso.
      self.language=False # Declara el idioma en español.
      # Declara los distintos subscriptores.
      self.lane_subscriber=rospy.Subscriber("/lane",Int32,self.laneCallback,queue_size=1)
      self.posel_subscriber=rospy.Subscriber("/tb3_0/amcl_pose", PoseWithCovarianceStamped,self.poseCallback,queue_size=1)
      self.posef_subscriber=rospy.Subscriber("/tb3_1/amcl_pose", PoseWithCovarianceStamped,self.poseCallback2,queue_size=1)
      self.graph_subscriber=rospy.Subscriber("/graph", Bool,self.graph_on_Callback,queue_size=1)
      self.clear_graph_subscriber=rospy.Subscriber("/clear_graph",Bool,self.clear_graph_Callback,queue_size=1)
      self.save_data_subscriber=rospy.Subscriber("/save_data",Bool,self.save_data_Callback,queue_size=1)
      self.change_language_subscriber=rospy.Subscriber("/change_language",Bool,self.change_language_Callback,queue_size=1)
      plt.ion() # Activa modo interactivo.

    def poseCallback(self,data):
        self.turtlebot3_pose_L.x=data.pose.pose.position.x
        self.turtlebot3_pose_L.y=data.pose.pose.position.y
        #self.Positions_XL.append(self.turtlebot3_pose_L.x)
        #self.Positions_YL.append(self.turtlebot3_pose_L.y)
        self.poseflag=True
    
    def poseCallback2(self,data):
        self.turtlebot3_pose_F.x=data.pose.pose.position.x
        self.turtlebot3_pose_F.y=data.pose.pose.position.y
        #self.Positions_X_F.append(self.turtlebot3_pose_F.x)
        #self.Positions_Y_F.append(self.turtlebot3_pose_F.y)
        self.poseflag=True

    def laneCallback(self,data):
        self.lane=data.data

    def graph_on_Callback(self,data):
        self.graph=data.data

    def clear_graph_Callback(self,data):
        self.clear=True

    def change_language_Callback(self,data):
        self.language=data.data
        
    def save_data_Callback(self,data):
        if self.count>2:
            self.save_data=True

    def plotear(self):
        self.rate = rospy.Rate(20)
        if self.save_data==True:
            self.save_data=False
            po=np.array(self.Positions_Count)
            xl=np.array(self.Positions_XL)
            yl=np.array(self.Positions_YL)
            el=np.array(self.Positions_EL)
            xf=np.array(self.Positions_XF)
            yf=np.array(self.Positions_YF)
            ef=np.array(self.Positions_EF)
            matrix=np.array([po,xl,yl,el,xf,yf,ef]).T
            ruta=os.path.dirname(os.path.abspath(__file__))
            nombre_Archivo='/Datos_de puntos_alcanzados/Grafica_'+time.strftime("%Y_%m_%d_%H_%M_%S")
            nombre_ArchivoP= nombre_Archivo+'.npy'
            nombre_ArchivoE= nombre_Archivo+'.xlsx'
            np.save(ruta+nombre_ArchivoP, matrix)
            df = pd.DataFrame(matrix,columns = ['Muestra','x Lider','y Lider','error Lider','x Seguidor','y Seguidor','error Seguidor'])
            df.to_excel(ruta+nombre_ArchivoE, sheet_name='Datos Obtenidos')
            print 'Datos guardados'
        
        if self.clear==True:
            self.clear=False
            self.count=0
            self.Positions_Count=[]
            self.Positions_XL=[]
            self.Positions_YL=[]
            self.Positions_EL=[]
            self.Positions_XF=[]
            self.Positions_YF=[]
            self.Positions_EF=[]
            plt.subplot(m,n,1)
            plt.cla()
            if self.language==True:
                plt.title('Reached Positions (Lider-Red,Follower-Blue)[m]')
            else:
                plt.title('Posiciones Alcanzadas (Lider-Rojo,Seguidor-Azul)[m]')
            plt.draw()
            plt.pause(0.00000000001)
            plt.subplot(m,n,2)
            plt.cla()
            if self.language==True:
                plt.title('Error Graph [m]')
            else:
                plt.title('Error Graficado [m]')
            plt.draw()
            plt.pause(0.00000000001)            
            print 'Grafica Limpiada'

        if self.graph==True:
            plt.subplot(m,n,1)
            plt.hold(True)
            #Posicion Seguidor
            plt.plot(self.turtlebot3_pose_L.x,self.turtlebot3_pose_L.y,'*r')
            #Posicion Lider
            plt.plot(self.turtlebot3_pose_F.x,self.turtlebot3_pose_F.y,'*b')
            plt.axis([-1.25,1.25,-1.25,1.25])
            if self.language==True:
                plt.title('Reached Positions (Lider-Red,Follower-Blue)[m]')
            else:
                plt.title('Posiciones Alcanzadas (Lider-Rojo,Seguidor-Azul)[m]')
            plt.draw()
            plt.pause(0.00000000001)
        
            if self.lane>0:
                #Almacenar Posiciones
                self.Positions_XL.append(self.turtlebot3_pose_L.x)
                self.Positions_YL.append(self.turtlebot3_pose_L.y)
                self.Positions_XF.append(self.turtlebot3_pose_F.x)
                self.Positions_YF.append(self.turtlebot3_pose_F.y)
                plt.subplot(m,n,2)
                plt.hold(True)
                #Error Lider
                distL, index = self.Arboles[self.lane-1].query((self.turtlebot3_pose_L.x,self.turtlebot3_pose_L.y))
                #Error Seguidor
                distF, index = self.Arboles[self.lane-1].query((self.turtlebot3_pose_F.x,self.turtlebot3_pose_F.y))
                self.Positions_EL.append(distL)
                self.Positions_EF.append(distF)
                self.Positions_Count.append(self.count)
                plt.plot(self.count,distL,'*r')
                plt.plot(self.count,distF,'*b')
                self.count=self.count+1
                plt.axis([0,self.count,0,0.8])
                if self.language==True:
                    plt.title('Error Graph ('+ str(distL) + ',' + str(distF) + ')[m]')
                else:
                    plt.title('Error Graficado('+ str(distL) + ',' + str(distF) + ')[m]')
                plt.draw()
                plt.pause(0.00000000001)
            else:
                plt.subplot(m,n,2)
                if self.language==True:
                    plt.title('Error Graph [m]')
                else:
                    plt.title('Error Graficado [m]')
                plt.pause(0.00000000001)
        self.rate.sleep()

    def Arbol(self,xy,longitud):
        ax,ay=xy.T
        l=len(ax)
        x=[]
        y=[]
        for i in range(l+1):
            ind1=i%l
            ind2=(i+1)%l
            xo=ax[ind1]
            xf=ax[ind2]
            diferenciaX=xf-xo
            yo=ay[ind1]
            yf=ay[ind2]
            diferenciaY=yf-yo
            for j in range(longitud):
                escala=float(j)/float(longitud)
                x.append(xo+diferenciaX*escala)
                y.append(yo+diferenciaY*escala)
        x1=np.array(x)
        y1=np.array(y)
        xy=np.array([x1,y1]).T
        return KDTree(xy)           

def main(): # Función principal
        rospy.init_node('Plotter', anonymous=True) # Inicia el nodo del graficador.
        P=Plotter() # Constructor de la clase Plotter.
        print 'PLotter inicializado' # Imprime mensaje de función principal inciada.
        while (not rospy.is_shutdown()):
            P.plotear() # Ir graficando los puntos.
        #rospy.spin()
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
