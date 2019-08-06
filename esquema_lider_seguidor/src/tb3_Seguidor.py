#!/usr/bin/env python
# encoding: utf-8
import numpy as np
import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16,Float64,Int32,Bool

follower_lin_vel=0.08
class FollowerRobot:
    def __init__(self):
        self.turn_on=False
        self.lim_angular=1.2
        self.wp=list()
        self.min_dist=3/100
        self.pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        self.sub_p_0 = rospy.Subscriber("/tb3_0/amcl_pose", PoseWithCovarianceStamped, self.LiderCallback, queue_size=1)  #Posicion lider
        self.sub_p_1 = rospy.Subscriber("/tb3_1/amcl_pose", PoseWithCovarianceStamped, self.FollowerCallback, queue_size=1)  #Posicion propia
        self.turn_on_subscriber=rospy.Subscriber("/turn_on",Bool,self.turn_onCallback,queue_size=1)
        self.map_size_x = 250 #cm
        self.map_size_y = 250 #cm
        self.resolution = 1 # cm
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0
        self.pub.publish(self.vel_msg)
        print 'nodo seguidor inicializado'
    
    def LiderCallback(self,data):
        #Hacer lista de los puntos por los que va pasando
        self.wp.insert(0,data)
        print 'Posicion lider recibida wp=',len(self.wp)
        #if  len(self.wp)< 40:
        #    self.vel_msg.linear.x = 0.0
        #    self.vel_msg.angular.z = 0.1
        #    self.pub.publish(self.vel_msg)

    def FollowerCallback(self, data):
        print 'Posicion seguidor recibida'
        if self.turn_on==True:
            wplen=len(self.wp)
            if  wplen> 60:
                #Aplicar waypoint algorithm
                #Datos del lider
                xL = self.wp[-1].pose.pose.position.x #Obtener la posicion mas antigua de x Lider.
                yL = self.wp[-1].pose.pose.position.y #Obtener la posicion mas antigua de y Lider.
                orientation_q = self.wp[-1].pose.pose.orientation #Obtener la orientacion mas antigua del Lider.
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (rollL, pitchL, yawL) = euler_from_quaternion (orientation_list)
            
                xF = data.pose.pose.position.x #Obtener posicion del seguidor
                yF = data.pose.pose.position.y #Obtener posicion del seguidor
                orientation_q = data.pose.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (rollF, pitchF, yawF) = euler_from_quaternion (orientation_list)
                print 'xF=',xF,'yF=',yF
                ka=1.5
                f_x=np.cos(yawF)*(xL-xF) + np.sin(yawF)*(yL-yF)
                f_y=-np.sin(yawF)*(xL-xF) + np.cos(yawF)*(yL-yF)
                a = np.arctan2(f_y,f_x)
                if a<0:
                    a=2*np.pi+a #convert from (-pi,pi), to (0,2pi)
                avel=(a+np.pi)%(2*np.pi)-np.pi
                if abs(avel)>45*np.pi/180:
                      lvel=0.0
                else:
                      lvel=follower_lin_vel
                print 'velocidad angular=',avel
                avel=ka*avel
                if avel>self.lim_angular:
                  avel=self.lim_angular
                elif avel<-self.lim_angular:
                  avel=-self.lim_angular
                self.vel_msg.linear.x = lvel
                self.vel_msg.angular.z = avel
                self.pub.publish(self.vel_msg)
                self.dist
                if self.dist<self.min_dist:
                    self.wp.pop()
            else:
                print 'Velocidad fija del movil'
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = 0.05
                self.pub.publish(self.vel_msg)

    def turn_onCallback(self,data):
        self.turn_on=data.data
        if self.turn_on==False:
            print 'Parar el movil'
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
            self.pub.publish(self.vel_msg)
        else:
            print 'Velocidad fija del movil'
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.05
            self.pub.publish(self.vel_msg)      
 
def main():
    rospy.init_node('FollowerRobot')
    FR=FollowerRobot()  # constructor creates publishers / subscribers
    print 'Nodo Seguidor inicializado'
    while (not rospy.is_shutdown()):
        FR.follow()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass