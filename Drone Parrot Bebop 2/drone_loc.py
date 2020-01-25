#!/usr/bin/env python
import rospy
import math
import time
import numpy
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point

class PID():
    def __init__(self, Kp, Kd, Ki, T, max_val, min_value):

        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        
      
        self.error = 0
        self.reference = 0
        self.prop = 0
        self.derivative = 0
        self.integral = 0
        # *************************
        
        self.u = 0
        #variables in which we save the values of errors that happened the step before
        self.Derivator_prior = 0
        self.Integral_prior = 0

        #iteration time
        self.T = T
        self.flag = True
        self.dozvoljeno = False
        #granicne vrijednosti
        self.max_val = max_val
        self.min_val = min_value
    

    def update(self,measured,reference):
       
        self.measured = measured
        self.reference = reference

        #calculate error
        self.error = self.reference - self.measured

        if abs(self.error) <= 0.1:
            self.dozvoljeno = True
        else:
            self.dozvoljeno = False
        
        #PID
        self.prop = self.error
        #derivator Euler backward 
        
      
        self.derivative = (1.0/self.T)*(self.error - self.Derivator_prior)
        #error from the last step (k-1)
        self.Derivator_prior = self.error
        #integrator
        self.integral = self.T * self.error + self.Integral_prior
        
        #upravljacka velicina
        self.u = self.Kp * self.prop + self.Kd * self.derivative + self.Ki * self.integral


        if(self.u > self.max_val):  
            self.u = self.max_val
            self.integral = self.Integral_prior 

        #negative saturation
        elif (self.u < self.min_val):
            self.u = self.min_val
            self.integral = self.Integral_prior

            

       
        self.Integral_prior = self.integral
   

        #vrati upravljacku velicinu
        return self.u, self.dozvoljeno


class Hover():

    def __init__(self):
        
        #publisher
        self.velocity_publisher = rospy.Publisher('/bebop/reg_mux',Twist,queue_size=1)
        self.nacin_rada = rospy.Publisher('/nacin',String,queue_size=1)
        self.dobar_kut = rospy.Publisher('/sphero',Bool,queue_size=1)

        #subscriber
        #self.odom_sub=rospy.Subscriber("odom",Odometry,self.PID_callback) #upisati ime topica od kamere
        self.centar_arene = rospy.Subscriber('/qr_point', Point, self.center_callback)
        self.centar_slike = rospy.Subscriber("/centar_slike", Point, self.slika_callback)
        self.zastavica=rospy.Subscriber("/qr_exist", Bool, self.zastavica_callback)
        self.controller = rospy.Subscriber("/bebop/joy",Joy,self.controller_callback)
        self.kut=rospy.Subscriber("/yaw",Point, self.yaw_callback)
        #define objects
        self.vel = Twist()
        self.centar_arene = Point()
        self.centar_slike = Point()
        #define variables
        self.freq = 30
        self.T = 1.0/30
        self.centar_arene.x = 0
        self.centar_arene.y = 0
        self.centar_slike.x = 0
        self.centar_slike.y = 0
        self.flag1 = True
        self.button = 0
        self.yaw_r  = 0.0
        self.yaw_m  = 0.0
        self.sphero_start = Bool()
        self.sphero_start = False
        self.zastavica_x  = False
        self.zastavica_y  = False
        self.zastavica_z  = False

        self.pid_pitch = PID(0.3, 0.277, 0, self.T, 1, -1)
        self.pid_roll = PID(-0.3, -0.2775, 0, self.T, 1, -1)
        self.pid_yaw = PID(2.83, 0, 0, self.T, 1, -1)
        
        self.rate = rospy.Rate(self.freq)
       
        
    
    def center_callback(self,data):
        # dobivanje koordinata centra arene
        self.centar_arene.x = data.x
        self.centar_arene.y = data.y
        
    def slika_callback(self,data):
        # dobivanje koordinata centra arene
        self.centar_slike.x = data.x
        self.centar_slike.y = data.y
    
    def zastavica_callback(self,data):

        self.flag1 = data.data            
    
    def controller_callback(self,data):
        self.button = data.buttons[7]
    
    def yaw_callback(self, data):
        self.yaw_m=data.x
            
    
    def run(self):

        while not rospy.is_shutdown():
            if self.flag1:

                ux, self.zastavica_x  = self.pid_pitch.update(self.centar_slike.x, self.centar_arene.x)
                uy, self.zastavica_y = self.pid_roll.update(self.centar_slike.y, self.centar_arene.y)
                uz, self.zastavica_z = self.pid_yaw.update(self.yaw_m, self.yaw_r)
            
            else:
                ux = 0
                uy = 0
                uz = 0 
                #if not self.button:
                    #self.nacin_rada.publish("regulator ne vidi centar, saljem nule")
            if not self.button:
                self.nacin_rada.publish("automatski nacin rada")
            else:
                self.nacin_rada.publish("rucni nacin rada")
            self.vel.linear.y = -ux
            self.vel.linear.x = uy
            if uz/2.83 < abs(0.039 * math.pi) and self.zastavica_x and self.zastavica_y and self.flag1:
                uz = 0
                self.sphero_start = True
            else:
                self.sphero_start = False
            self.vel.angular.z = -uz
            self.velocity_publisher.publish(self.vel)
            self.dobar_kut.publish(self.sphero_start)
            self.rate.sleep()
            


if __name__ == "__main__":

    rospy.init_node('drone_PID')

    try:
        ne = Hover()
        ne.run()
    except rospy.ROSInterruptException: pass


