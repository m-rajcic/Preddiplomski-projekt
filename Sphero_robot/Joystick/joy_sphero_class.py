#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Bool

class joy_sphero():
   
    def __init__(self):
        # publisheri
        self.pub = rospy.Publisher('/sphero_1/cmd_vel', Twist, queue_size= 1)
        self.pub2 = rospy.Publisher('/sphero_1/set_color', ColorRGBA, queue_size= 1)
        self.pub3 = rospy.Publisher('/sphero_1/manual_calibration', Bool, queue_size= 1)
        # stvaramo objekte
        self.vel = Twist()
        self.col = ColorRGBA()
        self.man = Bool()
        # varijable
        self.vel_x = 50
        self.vel_y = -50
        self.x_b = self.vel_x
        self.y_b = self.vel_y 
        # inicijalne vrijednosti 
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.col.r = 0
        self.col.g = 0
        self.col.b = 0
        self.man = False 

        self.flag = self.man
        self.flag1 = True

        # subscriber
        rospy.Subscriber("joy", Joy, self.joy_callback)

    
    def joy_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "\nGljiva x: %d \nGljiva y: %d", data.axes[0], data.axes[1])

        if data.buttons[7]:
            self.vel_x += 5
            self.vel_y = -self.vel_x
            if self.vel_x >= 250:
                self.vel_x = 250
                self.vel_y = -self.vel_x
            # cuvamo vrijednosti brzina da nakon "nitra" bude ista
            self.x_b = self.vel_x
            self.y_b = self.vel_y 
        if data.buttons[6]:
            self.vel_x -= 5
            self.vel_y = -self.vel_x
            if self.vel_x <= 10: 
                self.vel_x = 10
                self.vel_y = -self.vel_x
            # cuvamo vrijednosti brzina da nakon "nitra" bude ista
            self.x_b = self.vel_x
            self.y_b = self.vel_y 

        # "nitro"
        if data.buttons[10]:
            self.vel_x = 255
            self.vel_y = - self.vel_x
        else:
            self.vel_x = self.x_b
            self.vel_y = self.y_b
        
        # pridruzujemo vrijednosti za brzinu 
        self.vel.linear.x = self.vel_x * data.axes[0]
        self.vel.linear.y = self.vel_y * data.axes[1]

        #color setting - upravljanje bojama
        if data.buttons[0]:
            #self.col.r = 0
            #self.col.g = 0
            if not data.buttons[5]:
                self.col.b += 0.1
            else:
                self.col.b -= 0.1
        if data.buttons[1]:
            #self.col.r = 0
            if not data.buttons[5]:
                self.col.g += 0.1
            else:
                self.col.g -= 0.1
            #self.col.b = 0
        if data.buttons[2]:
            if not data.buttons[5]:
                self.col.r += 0.1
            else:
                self.col.r -= 0.1
            #self.col.g = 0
            #self.col.b = 0
        if data.buttons[3]:
            self.col.r = 0
            self.col.g = 0
            self.col.b = 0
        # zakretanje sphera
        if data.buttons[4]:
            self.man = True
            self.vel.linear.x = 0.5
            self.vel.linear.y = 0.5 
        else:
            self.man = False
        


    def run(self):
        # Main while loop.
        while not rospy.is_shutdown():
            # Publish our commands.
            self.pub.publish(self.vel)
            self.pub2.publish(self.col)
            if self.man != self.flag:
                self.pub3.publish(self.man)
                self.flag=self.man
            rospy.sleep(0.05)


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Joy_2_Sphero')
    # Go to class functions that do all the heavy lifting.
    try:
          ne = joy_sphero()  #definiran objekt klase NodeExample
          ne.run()
    except rospy.ROSInterruptException: pass
