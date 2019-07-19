#!/usr/bin/env python
import sys
import rospy
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from turtlesim.msg import Pose

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ForceController:
    
    def __init__(self):
      self.poseflag=False
      self.pose_subscriber=rospy.Subscriber("/tb3_0/amcl_pose", PoseWithCovarianceStamped,self.poseCallback,queue_size=1)
      self.turtlebot3_pose=Pose()
      self.goal_pose=Pose()
      self.vel_msg=Twist()
      #set in zero linear velocities
      self.vel_msg.linear.x=0
      self.vel_msg.linear.y=0
      self.vel_msg.linear.z=0
      #set in zero angular velocities
      self.vel_msg.angular.x=0
      self.vel_msg.angular.y=0
      self.vel_msg.angular.z=0
      #set map propierties
      self.map_size_x=250 #cm
      self.map_size_y=250 #cm
      self.resolution = 1 # cm
      self.lineal=0.07#.1 #velocidad lineal
      self.lim_angular=0.22 #limite velocidad angular.
      self.lane=2
      if (self.lane==1):
          self.matrix = np.load('/home/zarazua_rt/catkin_ws/src/tutoriales_basicos/scripts/TrayA1.npy')
      elif(self.lane==2):
          self.matrix = np.load('/home/zarazua_rt/catkin_ws/src/tutoriales_basicos/scripts/TrayB1.npy')
      else:
          self.matrix = np.load('/home/zarazua_rt/catkin_ws/src/tutoriales_basicos/scripts/TrayD1.npy')
      self.velocity_publisher=rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
      self.rate = rospy.Rate(10) # 10hz
      while self.poseflag==False:
          self.setStop()
      
    def poseCallback(self,data):
        self.turtlebot3_pose.x=data.pose.pose.position.x
        self.turtlebot3_pose.y=data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.turtlebot3_pose.theta=self.convert2pi(yaw)
        print 'yaw=',self.turtlebot3_pose.theta*180/np.pi
        self.poseflag=True               
    
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
              dist=self.getDistance2(self.turtlebot3_pose,goal_pose)
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
              self.vel_msg.angular.z =ka*(self.getAngle(self.turtlebot3_pose,goal_pose)-self.turtlebot3_pose.theta)
              self.velocity_publisher.publish(self.vel_msg)
              self.rate.sleep()
        #make zero the linear and angular velocity
        #self.vel_msg.linear.x=0
        #self.vel_msg.angular.z=0
        #self.velocity_publisher.publish(self.vel_msg)
        print 'x=',self.turtlebot3_pose.x,'y=',self.turtlebot3_pose.y

    def moveGoal2(self,goal_pose,distance_tolerance):
        self.rate = rospy.Rate(15)
        #****** Proportional Controller ******#
		#    linear velocity in the x-axis
        kl=0.3#0.22
        ka=1.5#1.5
        while True:
              dist=self.getDistance2(self.turtlebot3_pose,goal_pose)
              if not(dist>distance_tolerance):
                   break
              lvel=0.08#kl*dist
              if(lvel>0.22):
                  lvel=0.22
              a=(self.getAngle(self.turtlebot3_pose,goal_pose)-self.turtlebot3_pose.theta)
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
        print 'x=',self.turtlebot3_pose.x,'y=',self.turtlebot3_pose.y


    def follow(self):
        x1=self.turtlebot3_pose.x
        y1=self.turtlebot3_pose.y
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
        rospy.init_node('ForceController', anonymous=True)
        FC=ForceController() # constructor creates publishers / subscribers
        print 'Nodo inicializado'
        for i in range(0,100):
            FC.follow()
            print i
        FC.setStop()
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
