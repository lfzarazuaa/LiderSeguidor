#!/usr/bin/env python
# encoding: utf-8
import numpy as np
import rospy
import time
# Importar mensajes de los distintos paquetes para guardar variables en ellos.
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16,Float64,Int32,Bool

follower_lin_vel=0.08 # Velocidad lineal del robot seguidor.
map_size_x = 250.0 # Medida del mapa en "x" en cm.
map_size_y = 250.0 # Medida del mapa en "y" en cm.
resolution = 1.0 # Medida del mallado en cm.

class FollowerRobot: # Clase para el robot seguidor.
    def __init__(self): # Constructor de la clase.
        # Declara constantes que definen la acción del seguidor.
        self.map_size_x = float(map_size_x) # Medida del mapa en "x" en cm.
        self.map_size_y = float(map_size_y) # Medida del mapa en "y" en cm.
        self.resolution = float(resolution) # Medida del mallado en cm.
        self.follower_lin_vel = float(follower_lin_vel) # Velocidad lineal del robot seguidor.
        self.lim_angular = 1.2 # Límite de velocidad angular.
        # Declara variables útiles para seguir al líder y el movimiento del seguidor.
        self.turn_on = False # Variable para iniciar con el robot sin moverse.
        self.vel_msg = Twist() # Crea objeto para poder publicar la velocidad.
        self.vel_msg.linear.x, self.vel_msg.linear.y, self.vel_msg.linear.z = [0.0,0.0,0.0] # Declara la variable de la velocidad lineal en cero.
        self.vel_msg.angular.x,self.vel_msg.angular.y,self.vel_msg.angular.z = [0.0,0.0,0.0] # Declara la variable de la velocidad angular en cero.
        self.wp = list() # Crea una lista donde se guardan los puntos a alcanzar.
        # Crea Subscriptores
        self.sub_p_0 = rospy.Subscriber("/tb3_0/amcl_pose", PoseWithCovarianceStamped, self.LiderCallback, queue_size=1)  # Define método para obtener la posición del robot líder.
        self.sub_p_1 = rospy.Subscriber("/tb3_1/amcl_pose", PoseWithCovarianceStamped, self.FollowerCallback, queue_size=1)  # Define método obtener la posición del robot seguidor.
        self.turn_on_subscriber=rospy.Subscriber("/turn_on",Bool,self.turn_onCallback,queue_size=1) # Crea el subscriptor de avanzar o parar.
        # Crea Publicadores
        self.pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10) # Crea publicador de la velocidad que va a tener el robot.
        self.pub.publish(self.vel_msg) # Publica la velocidad inicial.
    
    def LiderCallback(self,data): # Método para obtener las posiciones del líder y guardarlas en una lista.
        # Hacer lista de los puntos por los que va pasando el robot líder
        self.wp.insert(0,data) # Crea lista de los puntos a alcanzar por el seguidor.
        #print 'Posicion lider recibida wp=',len(self.wp) # Imprime en pantalla la cantidad de puntos a seguir.

    def FollowerCallback(self, data): # Método para ejecutar cada vez que se recibe una nueva posición en el seguidor.
        #print 'Posicion seguidor recibida' # Imprime mensaje indicando que se recibió un mensaje.
        if self.turn_on==True: # Ejecuta rutina si se le indicó que avance.
            wplen=len(self.wp) # Guarda cuantas posiciones hay por alcanzar.
            if  wplen> 60: # Si es mayor a 60 posiciones ejecuta el algoritmo de seguimento de trayectoria esto para no chocar.
                # Aplicar el algoritmo waypoint
                # Datos del líder
                xL = self.wp[-1].pose.pose.position.x # Obtener la posición mas antigua de x Lider.
                yL = self.wp[-1].pose.pose.position.y # Obtener la posición mas antigua de y Lider.
                orientation_q = self.wp[-1].pose.pose.orientation # Obtener la orientacion mas antigua del Lider.
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] # Guardar los elementos de la orientación en una lista.
                (rollL, pitchL, yawL) = euler_from_quaternion (orientation_list) # Obtener la orientación del líder en ángulos esféricos en vez de cuaterniones.
                # Datos del seguidor
                xF = data.pose.pose.position.x # Obtener posición del seguidor
                yF = data.pose.pose.position.y # Obtener posición del seguidor
                orientation_q = data.pose.pose.orientation # Obtener la orientación del seguidor en cuaterniones.
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] # Guardar la orientación del seguidor en una lista.
                (rollF, pitchF, yawF) = euler_from_quaternion (orientation_list) # Obtener la orientación del seguidor en ángulos esféricos en vez de cuaterniones. 
                #print 'xF=',xF,'yF=',yF # Imprime la posición del seguidor.
                ka = 1.0 # Constante para incrementar la velocidad de giro.
                a = self.convert2pi(np.arctan2(yL-yF,xL-xF))-self.convert2pi(yawF) # Calcula la diferencia angular entre las 2 posciones.
                avel = (a+np.pi)%(2*np.pi)-np.pi # Orienta la velocidad en el sentido de giro mas corto.
                if abs(avel)>45*np.pi/180: # Si el ángulo a girar es mayor a 45 grados.
                      lvel = 0.0 # Rota sin velocidad lineal.
                else: # Si es menor.
                      lvel = self.follower_lin_vel # Rota con velocidad lineal constante.
                #print 'velocidad angular=',avel # Imprime el ángulo a rotar.
                avel = self.limitar(ka*avel,-self.lim_angular,self.lim_angular) # Obtiene la velocidad de giro contemplando su límite a alcanzar.
                self.vel_msg.linear.x = lvel # Guarda la velocidad lineal.
                self.vel_msg.angular.z = avel # Guarda la velocidad angular.
                self.pub.publish(self.vel_msg) # Publica la velocidad para que el robot seguidor para que vaya a la velocidad calculada.
                self.wp.pop() # Elimina la posición alcanzada.
            else: # Si aun no alcanza las posiciones deseadas.
                #print 'Velocidad fija del movil' # Imprime un mensaje que el robot seguidor aun no esta siguiendo.
                self.vel_msg.linear.x,self.vel_msg.angular.z = [0.0,0.05] # Guarda en la variable a publicar la velocidad para girar sobre su propio eje.
                self.pub.publish(self.vel_msg) # Publica la velocidad para que el robot seguidor gire sobre su propio eje.
 
    def turn_onCallback(self,data): # Método para que avance o se pare el robot seguidor.
        self.turn_on=data.data # Obtien si quiere que avance el robot seguidor.
        if self.turn_on==False: # Si no se quiere que avance el robot seguidor.
            #print 'Parar el movil' # Imprime el mensaje que se paró el robot seguidor.
            self.vel_msg.linear.x,self.vel_msg.angular.z = [0.0,0.05] # Guarda en la variable a publicar para parar al seguidor.
            self.pub.publish(self.vel_msg) # Publica la velocidad para que el seguidor va a tener.
        else: # Si se quiere que avance el robot seguidor.
            #print 'Velocidad fija del movil' # Imprime el mensaje que el robot seguidor siguiendo al lider.
            self.vel_msg.linear.x,self.vel_msg.angular.z = [0.0,0.05] # Guarda en la variable a publicar la velocidad para girar sobre su propio eje.
            self.pub.publish(self.vel_msg) # Publica la velocidad para que el seguidor va a tener.

    def limitar(self,x,lim_inf,lim_sup): # Metódo para fijar límites de un valor o función.
        if (x<=lim_inf): # Si es menor al límite inferior.
            x = lim_inf # Coloca el límite inferior.
        elif (x>=lim_sup): # Si es mayor al límite superior.
            x = lim_sup # Coloca el límite superior.
        return x

    def convert2pi(self,theta):# Método que convierte a un rango de 0 a 2pi.
        if theta<0: # Si el ángulo es negativo lo convierte a más de pi.
            theta=2*np.pi+theta
        else: # Deja el ángulo igual.
            theta=theta
        return theta # Regresa el ángulo ya convertido.

    def setStop(self): # Método para parar al robot.
        self.vel_msg.linear.x,self.vel_msg.angular.z=[0,0] # Establece las velocidades en 0.
        self.velocity_publisher.publish(self.vel_msg) # Publica la velocidad.

def main():
    rospy.init_node('FollowerRobot') # Inicia el nodo del robot seguidor.
    fr=FollowerRobot() # Constructor de la función del robot seguidor.
    print 'Nodo Seguidor inicializado' # Imprime que se logró inicializar el robot seguidor.
    rospy.spin() # Mientras ROS se siga ejecutando repite la secuencia.

if __name__ == '__main__':
    try:
        main() # Ejecuta el programa principal.
    except rospy.ROSInterruptException:
        pass