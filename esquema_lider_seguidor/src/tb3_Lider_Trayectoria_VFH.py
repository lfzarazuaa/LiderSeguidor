#!/usr/bin/env python
# encoding: utf-8
import sys 
import rospy
import numpy as np
import os
# Importar mensajes de los distintos paquetes para guardar variables en ellos.
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from turtlesim.msg import Pose
from std_msgs.msg import Float64,Int32,Bool
from esquema_lider_seguidor.msg import Histograma # Importa el mensaje histograma.

from tf.transformations import euler_from_quaternion, quaternion_from_euler
map_size_x = 250.0 # Medida del mapa en "x" en cm.
map_size_y = 250.0 # Medida del mapa en "y" en cm.
resolution = 1.0 # Medida del mallado en cm.
numero_de_trayectoria = 1 # Número de trayectoria seleccionda.
numero_de_iteraciones = 1000 # Número de iteraciones a realizar de seguimiento si no se usa GUI

class MovimientoLider: # Clase del movimiento del robot líder.
    def __init__(self): # Constructor de la clase o inicialización.
        self.inicializacion() # Inicia la función con los primeros parámetros.
        while self.poseflag==False: # Se mantiene parado mientras no haya recibido la primera posición del robot.
            self.setStop() # Coloca la velocidad del robot en 0.
    
    def inicializacion(self):
        # Definir los valores del mapa como dimensiones y resolucón del mallado.
        self.map_size_x = float(map_size_x) # Medida del mapa en "x" en cm.
        self.map_size_y = float(map_size_y) # Medida del mapa en "y" en cm.
        self.resolution = float(resolution) # Medida del mallado en cm.
        self.lineal = 0.08 # Velocidad lineal del robot.
        self.lim_angular = 1.2 # Límite velocidad angular del robot.
        self.lane = numero_de_trayectoria # Trayectoria seleccionada.
        self.numTrayectorias = 3 # Número de trayectorias.
        ruta = os.path.dirname(os.path.abspath(__file__))+'/Codigos_para_generacion_de_trayectorias/Archivos_de_Trayectoria/' # Obtener la ruta del archivo.
        nombre = 'MatrizDeFuerza' # Nombre del archivo de salida que contiene la matriz de fuerza.
        extension = '.npy' # Extensión del archivo.
        self.poseflag = False # Bandera de dato de posición obtenido. 
        self.histogramflag = False # Bandera de dato de histograma obtenido. 
        self.dist_minflag = False # Bandera de dato de distancia mínima al obstáculo obtenido. 
        self.lider_pose = Pose() # Inicializa el objeto de posición del robot líder.
        self.vel_msg = Twist() # Crea objeto para poder publicar la velocidad.
        # Variables para evadir obstáculos.
        self.obs_min_dist = 0.13+0.105*2.0 # Distancia a la que empieza a aplicar vfh.
        # Colocar en 0 las velocidades lineales.
        self.vel_msg.linear.x,self.vel_msg.linear.y,self.vel_msg.linear.z = [0,0,0]
        # Colocar en 0 las velocidades angulares.
        self.vel_msg.angular.x,self.vel_msg.angular.y,self.vel_msg.angular.z = [0,0,0]
        self.Trayectorias = [] # Inicializa la lista donde se guardan las trayectorias.
        for i in range(1,self.numTrayectorias+1): # Carga las trayectorias solicitadas.
            self.Trayectorias.append(np.load(ruta+nombre+str(i)+extension))
        # Se subscribe a distintos tópicos.
        self.pose_subscriber=rospy.Subscriber("/tb3_0/amcl_pose", PoseWithCovarianceStamped,self.poseCallback,queue_size=1) # Se subscribe a la posición del líder la cual se obtiene por amcl.
        self.histogram_subscriber=rospy.Subscriber("/tb3_0/histogram", Histograma,self.histogramCallback,queue_size=1) # Se subscribe al histograma que le sirve para esquivar obstáculos.
        self.dist_min_subscriber=rospy.Subscriber("/tb3_0/dist_min", Float64,self.dist_minCallback,queue_size=1) # Se subscribe a la distancia mínima hacia obstáculo para saber si aplicar vfh.
        # Publica el tópico de velocidad.
        self.velocity_publisher=rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10) # Crea publicador de la velocidad que va a tener el robot.
        self.rate = rospy.Rate(10) # Hace la frecuencia de espera de 10Hz cuando se usa ros sleep().
        
    def poseCallback(self,data): # Obtiene la posición del robot.
        self.lider_pose.x=data.pose.pose.position.x # Posición en "x" del robot.
        self.lider_pose.y=data.pose.pose.position.y # Posición en "y" del robot.
        orientation_q = data.pose.pose.orientation # Orientación en cuaternión del robot.
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]# Guarda en una lista la orientación.
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list) # Obtienen los 3 ángulos de orientación (x,y,z).
        self.lider_pose.theta=self.convert2pi(yaw) # Convierte el ángulo a un intervalo de 0 a 2pi.
        #print 'yaw=',self.lider_pose.theta*180/np.pi # Imprime la posición en pantalla.
        self.poseflag=True # Indica que ya se recibió la posición.
    
    def histogramCallback(self,data): # Método que se ejecuta cuando se recibe un Histograma.
        self.histogram=list(data.Histogram) # Guarda el histograma en una variable cuando es recibido
        self.histogramflag=True # Indica que ya se recibió el histograma.

    def dist_minCallback(self,data): # Método que se ejecuta cuando se recibe un Histograma.
        self.dist_min=data.data # Guarda la distancia más cercana a un posible obstáculo.
        self.dist_minflag=True # Indica que ya se recibió la distancia mínima.

    def laneCallback(self,data):# Guarda la opción de la trayectoria seleccionada
        self.lane=data.data

    def turn_onCallback(self,data):# Guarda la opción de avanzar o parar.
        self.turn_on=data.data
        if self.turn_on==False: # Si le indica parar. 
            self.setStop() # Para al robot.

    def esperardatos(self): # Verifica que las dos condiciones se cumplan para poder seguir sin errores.
        condicion=not(self.dist_minflag and self.histogramflag)
        while condicion:
            condicion=not(self.dist_minflag and self.histogramflag)

    def seguimientoSinGUI(self): # Sigue la tayectoria sin GUI
        self.esperardatos() # Espera que se escriba en las variables de histograma y distancia mínima.
        for i in range(0,numero_de_iteraciones):# Simula cierta cantidad de puntos alcanzados antes que se pare el robot lider.
                self.follow()# Llama a la función de seguir trayectoria.
                #print i # Imprime cuantos puntos ha alcanzado.
    
    def seguimientoConGUI(self): # Sigue la trayectoria manipulada por la GUI.
        self.esperardatos() # Espera que se escriba en las variables de histograma y distancia mínima.
        self.turn_on=False # Bandera que indica si avanza o se para.
        self.lane_subscriber = rospy.Subscriber("/lane",Int32,self.laneCallback,queue_size=1)# Crea el subscriptor de la Trayectoria escogida.
        self.turn_on_subscriber = rospy.Subscriber("/turn_on",Bool,self.turn_onCallback,queue_size=1)# Crea el subscriptor de avanzar o parar.
        while (not rospy.is_shutdown()):
            if self.turn_on==True and self.lane>0 and self.lane<=self.numTrayectorias:
                self.follow() # Llama a la función de seguir trayectoria.
            else:
                self.setStop() # Hace que el robot se pare.                 

    def convert2pi(self,theta):# Convierte a un rango de 0 a 2pi.
        if theta<0:# Si el ángulo es negativo lo convierte a más de pi.
            theta=2*np.pi+theta
        else:# Deja el ángulo igual.
            theta=theta
        return theta # Regresa el ángulo ya convertido.

    def limitar(self,x,lim_inf,lim_sup): #  Metódo para fijar límites de un valor o función.
        if (x<=lim_inf): # Si es menor al límite inferior.
            x = lim_inf # Coloca el límite inferior.
        elif (x>=lim_sup): # Si es mayor al límite superior.
            x = lim_sup # Coloca el límite superior.
        return x

    def setStop(self): # Para al robot
        self.vel_msg.linear.x,self.vel_msg.angular.z=[0,0] # Establece las velocidades en 0.
        self.velocity_publisher.publish(self.vel_msg) # Publica la velocidad.

    def move2angle(self,dist_y,dist_x): # Orienta la robot en el ángulo que indican las distancias de entrada al mismo tiempo que avanza.
        self.rate = rospy.Rate(10) # Máximo de veces se repite por segundo la función.
        ka=1.0 # Constante para incrementar la velocidad de giro.
        avel=self.convert2pi(np.arctan2(dist_y,dist_x))-self.lider_pose.theta # Calcula la diferencia angular entre las 2 posciones.
        avel=(avel+np.pi)%(2*np.pi)-np.pi # Orienta la velocidad en el sentido de giro mas corto.
        if abs(avel)>45*np.pi/180: # Si el giro es mayor a 45 grados hace que la velocidad lineal sea 0,
            lvel=0.0 # para evitar desviaciones largas provocadas por la velocidad lineal y pueda asi seguir mejor la trayectoria.
        else:
            lvel=self.lineal # Conserva la velocidad lineal constante.
        avel=ka*avel # Aumenta o disminuye la velocidad de giro.
        avel=self.limitar(avel,-self.lim_angular,self.lim_angular) # Límita la velocidad angular dentro de un margen para evitar giros bruscos.
        self.vel_msg.linear.x=lvel # Guarda la velocidad lineal.
        self.vel_msg.angular.z =avel # Guarda la velocidad angular.
        self.velocity_publisher.publish(self.vel_msg) # Publica la velocidad para que el robot vaya a la velocidad calculada.
        self.rate.sleep() # Ejecuta la pausa sin interrumpir otros procesos.

    def follow(self): # Calcula el siguiente punto a llegar y hace que el robot se oriente en dirección a esa posición.
        x1,y1=[self.lider_pose.x,self.lider_pose.y] # Lee la posición en "x" y en "y" del robot.
        x_index=np.int((x1+self.map_size_x/200)*(100/self.resolution)) # Obtener el índice en x para la matriz de fuerza.
        y_index=np.int((y1+self.map_size_y/200)*(100/self.resolution)) # Obtener el índice en y para la matriz de fuerza.
        # Definir límites del índice en "x" y en "y"
        x_index = self.limitar(x_index,0,self.map_size_x/self.resolution-1)
        y_index = self.limitar(y_index,0,self.map_size_y/self.resolution-1)
        if self.lane>0: # Si se seleccionó una trayectoria entonces se lee la trayectoria deseada.
            dist_x, dist_y = self.Trayectorias[self.lane-1][x_index,y_index,:] # Obtener la distancia a la que se quiere llegar.
        else:
            self.setStop() # Para el robot si no se seleccionó trayectoria.
            return
        self.an = int(self.convert2pi(np.arctan2(dist_y,dist_x))*180/np.pi) # Ángulo objetivo.
        if (self.dist_min>self.obs_min_dist) or (self.histogram[self.an]==0): # No Aplica VFH si no esta cerca o es un espacio libre.
            self.move2angle(dist_y,dist_x) # Establece la velocidad correcta para llegar al objetivo.
        else: # Aplica VFH encontrando el sector libre mas cercano al objetivo.
            self.VectorFieldHistogram() # Para el robot.

    def VectorFieldHistogram(self): # Método para aplicar la selección de ángulo libre.
        ind = self.an # Guarda el ángulo.
        h = self.histogram[ind] # Evalua si hay obstáculo en ese ángulo.
        while h==1: # Sigue avanzando hasta que no haya obstáculo.
            ind = (ind+1)%360 # Avanza en sentido antihorario.
            h = self.histogram[ind] # Indica si hay obstáculo.
        if self.dist_min<0.13+0.105*1.25: # Si la distancia es muy cercana
            ind = (ind+15)%360 # Hace un cambio de dirección mayor.
        ind = np.deg2rad(ind) # Convierte a degradianes el ángulo para aplicarle seno y coseno.
        self.move2angle(np.sin(ind),np.cos(ind)) # Establece la velocidad correcta para llegar al objetivo.
        
def main(): # Función principal
        rospy.init_node('MovimientoLider', anonymous=True) # Inicia el nodo MovimientoLider.
        gui = len(sys.argv)>3 # Decide si usar gui o no sin ningun argumento es con GUI.
        Lider = MovimientoLider() # Constructor de la clase donde se inicializan subscriptores y publicadores.
        print 'Nodo inicializado' # Se imprime para verificar que el nodo ya se inicializó.
        if gui:
            Lider.seguimientoSinGUI() # Ejecuta el Seguimiento sin necesidad de usar la GUI.
        else:
            Lider.seguimientoConGUI() # Ejecuta el Seguimiento de Trayectoria interfazandose con la GUI.
        Lider.setStop() # Para al robot.

if __name__ == '__main__':
    try:
        main() # Función principal
    except rospy.ROSInterruptException:
        pass