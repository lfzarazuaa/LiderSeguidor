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

class ForceController: #Clase del movimiento del robot líder.
    def __init__(self):#Constructor de la clase.
        self.poseflag=False #Bandera de dato de posición obtenido. 
        self.lider_pose=Pose()#Inicializa el objeto de posición del robot líder.
        self.goal_pose=Pose()#Posción a alcanzar.
        self.vel_msg=Twist()#Crea objeto para poder publicar la velocidad.
        #Colocar en 0 las velocidades lineales.
        self.vel_msg.linear.x=0
        self.vel_msg.linear.y=0
        self.vel_msg.linear.z=0
        #Colocar en 0 las velocidades angulares.
        self.vel_msg.angular.x=0
        self.vel_msg.angular.y=0
        self.vel_msg.angular.z=0
        #Definir los valores del mapa
        map_size_x=250.0 # Medida del mapa en "x" en cm.
        map_size_y=250.0 # Medida del mapa en "y" en cm.
        resolution = 1.0 # Medida del mallado en cm.
        self.lineal=0.07#.1 #Velocidad lineal del robot.
        self.lim_angular=0.13 #Límite velocidad angular.
        self.lane=2#Trayectoria Seleccionada.
        ruta = os.path.dirname(os.path.abspath(__file__))+'/Codigos_para_generacion_de_trayectorias/Archivos_de_Trayectoria/' #Obtener la ruta del archivo.
        nombre = 'MatrizDeFuerza'# Nombre del archivo de salida que contiene la matriz de fuerza.
        extension = '.npy'# Extensión del archivo.
        self.numTrayectorias = 3# Número de trayectorias.
        self.Trayectorias = []# Inicializa la lista donde se guardan las trayectorias.
        for i in range(1,self.numTrayectorias+1):#Carga las trayectorias solicitadas.
            self.Trayectorias.append(np.load(ruta+nombre+str(i)+extension))
        self.pose_subscriber=rospy.Subscriber("/tb3_0/amcl_pose", PoseWithCovarianceStamped,self.poseCallback,queue_size=1)#Se subscribe a la posición del líder la cual se obtiene por amcl.
        self.velocity_publisher=rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)# Publica la velocidad que va a tener el robot.
        self.rate = rospy.Rate(10) # Hace la frecuencia de espera de 10Hz cuando se usa ros rate.
        while self.poseflag==False:# Se mantiene parado mientras no haya recibido la primera posición del robot.
            self.setStop()# Coloca la velocidad del robot en 0.
      
    def poseCallback(self,data): # Obtiene la posición del robot.
        self.lider_pose.x=data.pose.pose.position.x# Posición en x del robot.
        self.lider_pose.y=data.pose.pose.position.y# Posición en y del robot.
        orientation_q = data.pose.pose.orientation# Orientación en cuaternión del robot.
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]# Guarda en una lista la orientación.
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)# Obtienen los 3 ángulos de orientación (x,y,z).
        self.lider_pose.theta=self.convert2pi(yaw)# Convierte el ángulo a un intervalo de 0 a 2pi.
        print 'yaw=',self.lider_pose.theta*180/np.pi #Imprime la posición en pantalla.
        self.poseflag=True# Indica que ya se recibió la posición.
    
    def convert2pi(self,theta):
        if theta<0:
            theta=2*np.pi+theta
        else:
            theta=theta
        return theta

    def setStop(self):
        self.vel_msg.linear.x=0
        self.vel_msg.angular.z=0
        self.velocity_publisher.publish(self.vel_msg)
    
    def getDistance(self,x1,x2,y1,y2):
    	return np.sqrt((x2-x1)**2+(y2-y1)**2)

    def getFlag(self):
        return self.poseflag

    def getDistance2(self,d1,d2):
        return np.sqrt(np.power(d2.x-d1.x,2)+np.power(d2.y-d1.y,2))

    def getAngle(self,d1,d2):
        return self.convert2pi(np.arctan2(d2.y-d1.y,d2.x-d1.x))

    def moveGoal(self,goal_pose,distance_tolerance):
        self.rate = rospy.Rate(15)
        #****** Proportional Controller ******#
		#    linear velocity in the x-axis
        kl=0.3#0.22
        ka=1.5#1.5
        while True:
              dist=self.getDistance2(self.lider_pose,goal_pose)
              if not(dist>distance_tolerance):
                   break
              xvel=kl*dist
              if(xvel>0.22):
                  xvel=0.22
              self.vel_msg.linear.x=xvel
              self.vel_msg.linear.y=0
              self.vel_msg.linear.z=0
              #set a ka proportional angular velocity in the z-axis
              self.vel_msg.angular.x=0
              self.vel_msg.angular.y=0
              self.vel_msg.angular.z =ka*(self.getAngle(self.lider_pose,goal_pose)-self.lider_pose.theta)
              self.velocity_publisher.publish(self.vel_msg)
              self.rate.sleep()
        #make zero the linear and angular velocity
        #self.vel_msg.linear.x=0
        #self.vel_msg.angular.z=0
        #self.velocity_publisher.publish(self.vel_msg)
        print 'x=',self.lider_pose.x,'y=',self.lider_pose.y

    def moveGoal2(self,goal_pose,distance_tolerance):
        self.rate = rospy.Rate(15)
        #****** Proportional Controller ******#
		#    linear velocity in the x-axis
        kl=0.3#0.22
        ka=1.5#1.5
        while True:
              dist=self.getDistance2(self.lider_pose,goal_pose)
              if not(dist>distance_tolerance):
                   break
              lvel=0.08#kl*dist
              if(lvel>0.22):
                  lvel=0.22
              a=(self.getAngle(self.lider_pose,goal_pose)-self.lider_pose.theta)
              avel=(a+np.pi)%(2*np.pi)-np.pi
              if abs(avel)>45*np.pi/180:
                  lvel=0
              self.vel_msg.linear.x=lvel
              self.vel_msg.linear.y=0
              self.vel_msg.linear.z=0
              #set a ka proportional angular velocity in the z-axis
              self.vel_msg.angular.x=0
              self.vel_msg.angular.y=0
              self.vel_msg.angular.z =ka*avel
              self.velocity_publisher.publish(self.vel_msg)
              self.rate.sleep()
        print 'x=',self.lider_pose.x,'y=',self.lider_pose.y


    def follow(self):
        x1=self.lider_pose.x
        y1=self.lider_pose.y
        x_index=np.int((x1+1.25)*(100/self.resolution))#obtener el indice en x
        y_index=np.int((y1+1.25)*(100/self.resolution))#obtener el indice en y
        #Definir limites del indice en x
        if (x_index<0):
            x_index = 0
        elif (x_index>((self.map_size_x/self.resolution)-1)):
            x_index=(self.map_size_x/self.resolution)-1

        #Definir limites del indice en y
        if (y_index<0):
            y_index = 0
        elif (y_index>((self.map_size_y/self.resolution)-1)):
            y_index=(self.map_size_y/self.resolution)-1

        dist_x, dist_y = self.matrix[x_index,y_index,:]#Obtener la distancia a la que se quiere llegar.
        x2=x1+dist_x/2
        y2=y1+dist_y/2
        dist=self.getDistance(x1,x2,y1,y2)
        min_dist=0.025
        if dist<min_dist:
            min_dist=dist/2
            print 'min_dist=',min_dist
        print 'x1=',x1,'x2=',x2,'y1=',y1,'y2=',y2
        self.goal_pose.x=x2
        self.goal_pose.y=y2
        self.moveGoal2(self.goal_pose,min_dist)#0.025

def main():
        rospy.init_node('ForceController', anonymous=True)#Inicia el nodo ForceController.
        FC=ForceController() # constructor de la clase donde se inicializan subscriptores y publicadores.
        print 'Nodo inicializado'#Se imprime para verificar que el nodo ya se inicializó.
        #for i in range(0,100):#Simula cierta cantidad de puntos alcanzados antes que se pare el robot lider.
        #    FC.follow()#Funcion de seguir trayectoria.
        #    print i #Imprime cuantos puntos ha alcanzado.
        #FC.setStop()#Para al robot.
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
