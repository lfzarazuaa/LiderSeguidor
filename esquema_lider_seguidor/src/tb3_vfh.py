#!/usr/bin/env python
# encoding: utf-8
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt
#Importar mensajes de los distintos paquetes para guardar variables en ellos.
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from turtlesim.msg import Pose
from esquema_lider_seguidor.msg import Histograma # Importa el mensaje histograma.

from tf.transformations import euler_from_quaternion, quaternion_from_euler

min_radius = 45.0 # Distancia en cm a la que es considerado un obstáculo cercano.
s=30 # Grados a ensanchar el Histograma.
class LaserVFH: # Clase para realizar el algoritmo vector field histogram.
    def __init__(self):
        self.poseflag=False # Bandera de dato de posición obtenido.
        self.lidar_err=0.004 # Distancia de error del lidar para evitar ver obstáculos falsos.
        self.lider_pose=Pose() # Inicializa el objeto de posición del robot líder. 
        self.pose_sub = rospy.Subscriber("/tb3_0/amcl_pose", PoseWithCovarianceStamped, self.poseCallback, queue_size=1)# Se subscribe a la posición del líder la cual se obtiene por amcl.
        self.laser_sub = rospy.Subscriber("/tb3_0/scan", LaserScan, self.LaserCallback, queue_size=1)  # Ejecuta la conversión al histograma de vfh cuando se recibe una muestra del láser.
        self.histogram_publisher = rospy.Publisher('/tb3_0/histogram', Histograma, queue_size=1) # Crea variable para poder publicar el histograma de vfh.
        self.histogram = Histograma() # Crea variable para guardar el histograma.
        self.an = range(0,360,1) # Vector que representa los 360 grados.
        self.H = np.zeros(360,dtype='f') # Histograma de unos y ceros que indica la presencia de obstáculos.
        self.Ho = np.zeros(360,dtype='f') # Histograma de unos y ceros que indica la presencia de obstáculos.
        self.hp = np.zeros(360,dtype='f') # Derivada del histograma.
        plt.ion() # Activa el modo interactivo del plot.
        self.plot=False # Bandera para graficar los histogramas.
        
    def poseCallback(self,data): # Obtiene la posición del robot.
        self.lider_pose.x = data.pose.pose.position.x # Posición en "x" del robot.
        self.lider_pose.y = data.pose.pose.position.y # Posición en "y" del robot.
        orientation_q = data.pose.pose.orientation # Orientación en cuaternión del robot.
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] # Guarda en una lista la orientación.
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list) # Obtienen los 3 ángulos de orientación (x,y,z).
        self.lider_pose.theta=np.round(self.convert2pi(yaw)*180/np.pi) # Convierte el ángulo a un intervalo de 0 a 2pi.
        self.poseflag = True # Indica que ya se recibió la posición.       
               
    def LaserCallback(self,data): # Método para obtener el histograma de vfh.
        if self.poseflag==False:
            return
        # Genera el histograma original.
        for i in self.an: # Función que itera sobre las 360 distancias que da el lidar.
            dist = data.ranges[self.obtainIndexAngle(i)]*100 # Obtiene la distancia en cm del ángulo absoluto que se le pide. 
            if dist<=min_radius and dist>=self.lidar_err: # Verifica si es una distancia de un posible obstáculo.
                self.H[i],self.Ho[i] = [1,1] # Guarda el dato como obstáculo. 
            else:
                self.H[i],self.Ho[i] = [0,0] # Guarda el dato como espacio libre.
        # Obtiene la derivada del histograma.
        for i in self.an: # Calcula la derivada para los 360 grados.
            self.hp[i]=self.H[i]-self.H[(i-1)%360] # Se incluye el módulo para el caso de restar H[0]-H[359]
        # Encuentra valles y crestas para ensanchar el histograma original.
        self.crest = [] # Registro de crestas.
        self.trough = [] # Registro de valles.
        # Encuenta los valles y crestas.
        for i in self.an: # Encuentra los valles y crestas.
            if self.hp[i]>0.5: # Si la derivada es mayor a 0.5 es un incremento grande.
                self.crest.append(i) # Guarda en una lista la ubicación de todos las crestas.
            elif self.hp[i]<-0.5: # Si la derivada es menor a -0.5 es un decremento grande.
                self.trough.append(i) # Guarda en una lista la ubicación de todos los valles.
        # Ensancha el histograma polar.
        l = len(self.crest) # Verifica cuantas crestas se tuvieron.
        if l>0: # Si hay crestas.
            for i in range(0,l,1): # Itera sobre cada cresta.
                ind_back = self.crest[i] # Lee la ubicación de la cresta.
                for k in range (0,-s,-1): # Ensancha la cresta "s" grados hacia la izquierda.
                    self.H[(ind_back+k)%360] = 1 # Marca en 1 para ensanchar en histograma.
        l = len(self.trough)
        if l>0: # Si hay valles.
            for i in range(0,l,1): # Itera sobre cada valle.
                ind_forw = self.trough[i] # Lee la ubicación del valle.
                for k in range (0,s,1): # Ensancha los valles "s" grados hacia la derecha.
                    self.H[(ind_forw+k)%360] = 1 # Marca en 1 para ensanchar en histograma.
        self.histogram = self.H # Guarda el histograma en variable para publicarlo.
        # Publica el histograma ensanchado.
        self.histogram_publisher.publish(self.histogram) # Publica el histograma.
        # Grafica el Histograma.
        if self.plot==True: # Decide si grafica los Histogramas.
            m,n=[2,1] # Divide la gráfica en 2.
            plt.subplot(m,n,1) # Elige la primera gráfica.
            plt.hold(False) # No guarda la gráfica pasada.
            plt.plot(self.an,self.Ho) # Gráfica el histograma original.
            plt.axis([0,360,-2,2]) # Establece los límites de la gráfica.
            plt.title("Histograma Polar") # Le coloca un título a la gráfica.
            plt.draw() # Hace visible la gráfica. 
            plt.pause(0.00000000001) # Le da un tiempo para gráficar.
            plt.subplot(m,n,2) # Elige la primera gráfica.
            plt.hold(False) # No guarda la gráfica pasada.
            plt.plot(self.an,self.H) # Gráfica el histograma modificado.
            plt.axis([0,360,-1.5,1.5]) # Establece los límites de la gráfica.
            plt.title("Histograma Polar Modificado") # Le coloca un título a la gráfica.
            plt.draw() # Hace visible la gráfica. 
            plt.pause(0.00000000001) # Le da un tiempo para gráficar.

    def convert2pi(self,theta):# Convierte a un rango de 0 a 2pi.
        if theta<0:# Si el ángulo es negativo lo convierte a más de pi.
            theta=2*np.pi+theta
        else:# Deja el ángulo igual.
            theta=theta
        return theta # Regresa el ángulo ya convertido.

    def obtainIndexAngle(self,angle): # Obtiene el índice correcto.
        index=(angle-int(self.lider_pose.theta))%360 # Ajusta el ángulo al marco de referencia absoluto.
        return index

def main():
        rospy.init_node('Laser_VFH', anonymous=True)
        L=LaserVFH() # Constructor inicializar el nodo vfh.
        rospy.spin() # Se ejecuta hasta que ros-master se cierre.
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass