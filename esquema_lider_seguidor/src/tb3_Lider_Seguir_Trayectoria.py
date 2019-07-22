#!/usr/bin/env python
# encoding: utf-8
import sys 
import rospy
import numpy as np
import os
#Importar mensajes de los distintos paquetes para guardar variables en ellos.
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from turtlesim.msg import Pose

from tf.transformations import euler_from_quaternion, quaternion_from_euler
map_size_x = 250.0 # Medida del mapa en "x" en cm.
map_size_y = 250.0 # Medida del mapa en "y" en cm.
resolution = 1.0 # Medida del mallado en cm.
class MovimientoLider: # Clase del movimiento del robot líder.
    def __init__(self):# Constructor de la clase o inicialización.
        # Definir los valores del mapa como dimensiones y resolucón del mallado.
        self.map_size_x = float(map_size_x) # Medida del mapa en "x" en cm.
        self.map_size_y = float(map_size_y) # Medida del mapa en "y" en cm.
        self.resolution = float(resolution) # Medida del mallado en cm.
        self.lineal = 0.08 # Velocidad lineal del robot.
        self.lim_angular = 1.2 # Límite velocidad angular del robot.
        self.lane = 3# Trayectoria seleccionada.
        self.numTrayectorias = 3# Número de trayectorias.
        ruta = os.path.dirname(os.path.abspath(__file__))+'/Codigos_para_generacion_de_trayectorias/Archivos_de_Trayectoria/' # Obtener la ruta del archivo.
        nombre = 'MatrizDeFuerza'# Nombre del archivo de salida que contiene la matriz de fuerza.
        extension = '.npy'# Extensión del archivo.
        self.poseflag = False # Bandera de dato de posición obtenido. 
        self.lider_pose = Pose() # Inicializa el objeto de posición del robot líder.
        self.vel_msg = Twist() # Crea objeto para poder publicar la velocidad.
        # Colocar en 0 las velocidades lineales.
        self.vel_msg.linear.x,self.vel_msg.linear.y,self.vel_msg.linear.z = [0,0,0]
        # Colocar en 0 las velocidades angulares.
        self.vel_msg.angular.x,self.vel_msg.angular.y,self.vel_msg.angular.z = [0,0,0]
        self.Trayectorias = []# Inicializa la lista donde se guardan las trayectorias.
        for i in range(1,self.numTrayectorias+1):# Carga las trayectorias solicitadas.
            self.Trayectorias.append(np.load(ruta+nombre+str(i)+extension))
        self.pose_subscriber=rospy.Subscriber("/tb3_0/amcl_pose", PoseWithCovarianceStamped,self.poseCallback,queue_size=1)# Se subscribe a la posición del líder la cual se obtiene por amcl.
        self.velocity_publisher=rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)# Crea publicador de la velocidad que va a tener el robot.
        self.rate = rospy.Rate(10) # Hace la frecuencia de espera de 10Hz cuando se usa ros sleep().
        while self.poseflag==False:# Se mantiene parado mientras no haya recibido la primera posición del robot.
            self.setStop()# Coloca la velocidad del robot en 0.
      
    def poseCallback(self,data): # Obtiene la posición del robot.
        self.lider_pose.x=data.pose.pose.position.x# Posición en "x" del robot.
        self.lider_pose.y=data.pose.pose.position.y# Posición en "y" del robot.
        orientation_q = data.pose.pose.orientation# Orientación en cuaternión del robot.
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]# Guarda en una lista la orientación.
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)# Obtienen los 3 ángulos de orientación (x,y,z).
        self.lider_pose.theta=self.convert2pi(yaw)# Convierte el ángulo a un intervalo de 0 a 2pi.
        #print 'yaw=',self.lider_pose.theta*180/np.pi # Imprime la posición en pantalla.
        self.poseflag=True# Indica que ya se recibió la posición.
    
    def convert2pi(self,theta):# Convierte a un rango de 0 a 2pi.
        if theta<0:# Si el ángulo es negativo lo convierte a más de pi.
            theta=2*np.pi+theta
        else:# Deja el ángulo igual.
            theta=theta
        return theta

    def limitar(self,x,lim_inf,lim_sup):# Función para fijar límites de un valor o función.
        if (x<=lim_inf):# Si es menor al límite inferior.
            x = lim_inf # Coloca el límite inferior.
        elif (x>=lim_sup):# Si es mayor al límite superior.
            x = lim_sup# Coloca el límite superior.
        return x

    def setStop(self):# Para al robot
        self.vel_msg.linear.x,self.vel_msg.angular.z=[0,0]# Establece las velocidades en 0.
        self.velocity_publisher.publish(self.vel_msg)# Publica la velocidad.

    def move2angle(self,dist_y,dist_x):#Orienta la robot en el ángulo que indican las distancias de entrada al mismo tiempo que avanza.
        self.rate = rospy.Rate(10)# Maximo de veces se repite por segundo la función.
        ka=1.0# Constante para incrementar la velocidad de giro.
        avel=self.convert2pi(np.arctan2(dist_y,dist_x))-self.lider_pose.theta# Calcula la diferencia angular entre las 2 posciones.
        avel=(avel+np.pi)%(2*np.pi)-np.pi# Orienta la velocidad en el sentido de giro mas corto.
        if abs(avel)>45*np.pi/180:# Si el giro es mayor a 45 grados hace que la velocidad lineal sea 0,
            lvel=0.0 # para evitar desviaciones largas provocadas por la velocidad lineal y pueda asi seguir mejor la trayectoria.
        else:
            lvel=self.lineal# Conserva la velocidad lineal constante.
        avel=ka*avel# Aumenta o disminuye la velocidad de giro.
        avel=self.limitar(avel,-self.lim_angular,self.lim_angular)# Límita la velocidad angular dentro de un margen para evitar giros bruscos.
        self.vel_msg.linear.x=lvel# Guarda la velocidad lineal.
        self.vel_msg.angular.z =avel# Guarda la velocidad angular.
        self.velocity_publisher.publish(self.vel_msg)# Publica la velocidad para que el robot vaya a la velocidad calculada.
        self.rate.sleep()# Ejecuta la pausa sin interrumpir otros procesos.

    def follow(self):#Calcula el siguiente punto a llegar y hace que el robot se oriente en dirección a esa posición.
        x1,y1=[self.lider_pose.x,self.lider_pose.y]# Lee la posición en "x" y en "y" del robot.
        x_index=np.int((x1+self.map_size_x/200)*(100/self.resolution))# Obtener el índice en x para la matriz de fuerza.
        y_index=np.int((y1+self.map_size_y/200)*(100/self.resolution))# Obtener el índice en y para la matriz de fuerza.
        # Definir límites del índice en "x" y en "y"
        x_index = self.limitar(x_index,0,self.map_size_x/self.resolution-1)
        y_index = self.limitar(y_index,0,self.map_size_y/self.resolution-1)
        if self.lane>0:# Si se seleccionó una trayectoria entonces se lee la trayectoria deseada.
            dist_x, dist_y = self.Trayectorias[self.lane-1][x_index,y_index,:]# Obtener la distancia a la que se quiere llegar.
        else:
            self.setStop()# Para el robot si no se seleccionó trayectoria.
            return
        self.move2angle(dist_y,dist_x)# Establece la velocidad correcta para llegar al objetivo.

def main():
        rospy.init_node('MovimientoLider', anonymous=True)#Inicia el nodo ForceController.
        Lider=MovimientoLider() # Constructor de la clase donde se inicializan subscriptores y publicadores.
        print 'Nodo inicializado'# Se imprime para verificar que el nodo ya se inicializó.
        for i in range(0,1000):# Simula cierta cantidad de puntos alcanzados antes que se pare el robot lider.
            Lider.follow()# Función de seguir trayectoria.
            print i # Imprime cuantos puntos ha alcanzado.
        Lider.setStop()# Para al robot.
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass