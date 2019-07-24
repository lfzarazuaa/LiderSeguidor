#!/usr/bin/env python
# encoding: utf-8
import rospy
# Importar mensajes de los distintos paquetes para guardar variables en ellos.
from sensor_msgs.msg import LaserScan # Mensaje de información del lidar.
from std_msgs.msg import Float64 # Variable para guardar la distancia mínima al obstáculo.

class Obstacle(): # Clase de detección de obstáculos 
    def __init__(self): # Incialización de la clase.
        self.min_dist_publisher=rospy.Publisher('/tb3_0/dist_min', Float64, queue_size=10) # Variable publicadora de distancia mínima como tópico.
        self.lidar_err = 0.004 # Distancia de error del lidar para evitar ver obstáculos falsos.
        self.dist_min = Float64() # Crea objeto para guardar la distancia mínima.
        self.rate = rospy.Rate(30) # Configura el tiempo de muestreo.
        self.obtain_min_distance() # Ejecuta método de detección de obstáculos
        
    def get_scan(self): # Método para escanear las distancias que da el lidar.
        msg = rospy.wait_for_message("/tb3_0/scan", LaserScan) # Espera a la llegada de una nueva muestra.
        self.scan_filter = [] # Crea lista vacia para guardar distancias de posible obstáculo.
        self.scan_filter_err = [] # Crea lista para guardar las demás distancias.
        for i in range(360): # Verifica cada grado escaneado por el sensor lidar.
            if i <= 135 or i > 225: # Obtiene las distancias de los ángulos que se quieran.
                if msg.ranges[i] >= self.lidar_err: # Valida si es una distancia no errónea.
                    self.scan_filter.append(msg.ranges[i]) # Guarda todos los ángulos.
                else:
                    self.scan_filter_err.append(msg.ranges[i]) # Guarda los datos con error.
                    
    def obtain_min_distance(self): # Método de obtención de distancia mínima cíclico.
        while not rospy.is_shutdown(): # Mientras se ejecute Ros se sigue ejecutando.
            self.get_scan() # Obtiene las distancias del lidar que pueden ser obstáculo.
            self.dist_min = min(self.scan_filter) # Obtiene la distancia mínima al obstáculo.
            self.min_dist_publisher.publish(self.dist_min) # Publica la distancia mínima.

def main():
    rospy.init_node('turtlebot3_distance') #Inicializa el nodo.
    try:
        obstacle = Obstacle() # Inicializa la clase.
    except rospy.ROSInterruptException:
        pass # Ejecuta la exepción.

if __name__ == '__main__':
    main()