#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

# za visinu drona 215cm ispada da je 1m cca (380-140 piksela = 240 piksela)

class PID_controller():  # mozda bi trebao odjednom obje tocke racunati bolje bude ti 

    def __init__(self, K_p, K_i, K_d, T, min_val, max_val):
        # pojacanja
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d # u ovom slucaju ga ni ne koristimo pa nam ne treba
        # pogreska
        self.error = 0
        # vrijednosti regulatora
        self.prop = 0
        self.intg = 0
        self.der  = 0
        # vrijednosti iz koraka k-1
        self.Derivator_prior = 0
        self.Integral_prior  = 0
        self.error_prior     = 0
        # vrijeme diskretizacije
        self.T = T
        # maks i min brzina
        self.max_val = max_val
        self.min_val = min_val
    
    def find_values(self, finish_point, reference_point):
        self.goto_point = finish_point
        self.reference_point = reference_point
        # trenutna greska
        self.error = self.reference_point - self.finish_point    # oduzimanje zeljene i trenutne x pozciije

        # racunanje brzine 
        self.prop  = self.error 
        self.der   = (2.0 / self.T) * (self.error - self.error_prior) - self.Derivator_prior
        self.intg  = (T / 2.0) * (self.error + self.error_prior) + self.Integral_prior
        self.Derivator_prior = self.der
        self.error_prior     = self.error

        # konacna vrijednost iz regulatora
        self.final_value = self.prop + (self.K_i * self.intg) + (self.K_d * self.der)
            
        # podrucja premalih ili prevelikih brzina
        if self.final_value > 0:
            if self.final_value < self.min_val:
                self.final_value = self.min_val
                self.intg = self.Integral_prior 
            elif self.final_value > self.max_val:
                self.final_value = self.max_val
                self.intg = self.Integral_prior 
        elif self.final_value < 0:
            if self.final_value > -self.min_val:
                self.final_value = -self.min_val
                self.intg = self.Integral_prior 
            elif self.final_value < -self.max_val:
                self.final_value = -self.max_val
                self.intg = self.Integral_prior 
            
        self.Integral_prior = self.intg

        # podrucje u kojem gubi brzinu - mrtva zona
        if abs(self.error) < 15:
            self.final_value = 0
        if abs(self.error) < 15:
            self.final_value = 0

        return self.final_value

class sphero_loc():

    def __init__(self):
        # publisheri
        self.pub1 = rospy.Publisher('/sphero_1/cmd_vel', Twist, queue_size= 1)
        self.pub2 = rospy.Publisher('/sphero_2/cmd_vel', Twist, queue_size= 1)
        # stvaramo objekte
        self.vel = Twist()
        self.pose_m = Point()
        # varijable i inicijalne vrijednosti
        self.vel.angular.x = 0
        self.vel.angular.y = 0

        self.pose_m.x = 0
        self.pose_m.y = 0

        self.pose_x_R_S = 0
        self.pose_y_R_S = 0
        self.pose_x_B_S = 0
        self.pose_y_B_S = 0

        self.pose_R_x = 0
        self.pose_R_y = 0 
        self.pose_B_x = 0
        self.pose_B_y = 0
        # parametri regulatora
        self.K_p = 0.6      # proporcionalno pojacanje
        self.K_i = 0.0005   # integralno pojacanje
        self.K_d = 0        # derivacijsko pojacanje
        self.T = 0.15
        self.max = 180
        self.min = 10

        self.v_x = 0
        self.v_y = 0
        # pomocni brojac
        self.cnt = 0

        # objekt razreda PID_controller
        self.pid_sphero_x = PID_controller(self.K_p, self.K_i, self.K_d, self.T, self.min, self.max)
        self.pid_sphero_y = PID_controller(self.K_p, self.K_i, self.K_d, self.T, self.min, self.max)

        # dobivanje pozicije crvenog markera --> samo jednom
        self.pose_m = rospy.wait_for_message("/red_m", Point)
        self.pose_R_x = self.pose_m.x
        self.pose_R_y = self.pose_m.y
        # dobivanje pozicije plavog markera --> samo jednom
        self.pose_m = rospy.wait_for_message("/blue_m", Point)
        self.pose_B_x = self.pose_m.x
        self.pose_B_y = self.pose_m.y

        # subscriberi
        rospy.Subscriber("/red_s", Point, self.sphero_red_callback)
        rospy.Subscriber("/blue_s", Point, self.sphero_blue_callback)

        # konacna brzina
        self.final_value = 0

    def sphero_red_callback(self, data):
        self.pose_x_R_S = data.x
        self.pose_y_R_S = data.y

    def sphero_blue_callback(self, data):
        self.pose_x_B_S = data.x
        self.pose_y_B_S = data.y

    def run(self):

        r = rospy.Rate(1/self.T)
        while not rospy.is_shutdown():
            # crveni sphero
            if self.cnt == 0:
                self.vel_x = self.pid_sphero_x.find_values(self.pose_R_x, self.pose_x_R_S)
                self.vel.linear.x = self.v_x
                self.vel_y = self.pid_sphero_y.find_values(self.pose_R_y, self.pose_y_R_S)
                self.vel.linear.y = self.vel_y 
                self.pub1.publish(self.vel)
                print("Crveni v_x = {}, v_y = {}".format(self.vel_x, self.vel_y))
                if self.vel.linear.x == 0 and self.vel.linear.y == 0:
                    self.cnt += 1
            # plavi sphero
            elif self.cnt == 1:
                self.vel_x = self.pid_sphero_x.final_value(self.pose_B_x, self.pose_x_B_S)
                self.vel.linear.x = self.vel_x
                self.vel_y = self.pid_sphero_y.final_value(self.pose_B_y, self.pose_y_B_S)
                self.vel.linear.y = -self.vel_y
                print("Plavi v_x = {}, v_y = {}".format(self.vel_x, self.vel_y))                
                self.pub2.publish(self.vel)
                if self.vel.linear.x == 0 and self.vel.linear.y == 0:
                    self.cnt += 1
            # kraj
            else:
                print("Oba Sphero SPRK+ sferna robota su na svojem mjestu pod suncem.")
                exit() # izadi iz programa

            r.sleep()


if __name__ == '__main__':
    # inicijalizacija cvora
    rospy.init_node('Sphero_LOC')
    try:
          ne = sphero_loc()
          ne.run()
    except rospy.ROSInterruptException: pass