#!/usr/bin/env python2.7
  # Import ROS libraries and messages
import rospy
import time
import pyzbar.pyzbar as pyzbar
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(1)

class Position(object):

    def __init__(self):
        print('Inicijalizacija klase')
        self.red_s = rospy.Publisher("/red_s",Point,queue_size = 1)
        self.red_m = rospy.Publisher("/red_m",Point,queue_size = 1)
        self.blue_s = rospy.Publisher("/blue_s",Point,queue_size = 1)
        self.blue_m = rospy.Publisher("/blue_m",Point,queue_size = 1)
        self.qr_point = rospy.Publisher("/qr_point",Point,queue_size = 1)
        # objekti
        self.point_R_m = Point()
        self.point_R_s = Point()  
        self.point_B_m = Point()
        self.point_B_s = Point() 
        self.point_QR  = Point() 
        # inicijalizacija varijabli
        self.point_R_m.x = 0
        self.point_R_m.y = 0
        self.point_R_s.x = 0
        self.point_R_s.y = 0
        self.point_B_m.x = 0
        self.point_B_m.y = 0
        self.point_B_s.x = 0
        self.point_B_s.y = 0
        self.Xvc  = 0
        self.Yvc  = 0
        self.Xmc  = 0
        self.Ymc  = 0
        self.Xvp  = 0
        self.Yvp  = 0
        self.Xmp  = 0
        self.Ymvp = 0
        self.point_QR.x  = 0
        self.point_QR.y  = 0
        # qr kod
        self.sredinaX = 0
        self.sredinaY = 0
        # NAMJESTI PRIJE TESTIRANJA NA NEKU VRIJEDNOST
        self.pix = 1.0

        rospy.Subscriber("/bebop/image_raw", Image, self.image_callback, queue_size = 1, buff_size = 2**30)


    def image_callback(self, img_msg):
        print("Callback")
        now1 = rospy.get_rostime()

        rospy.loginfo(img_msg.header)

        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        im_bgr=cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        dimension = im_bgr.shape
        print(dimension)
        gray = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2GRAY)
         # _, th1 = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
        decodedObject = pyzbar.decode(gray)

        
        if( len(decodedObject) == 2):
            # print(decodedObject[0].rect[2]) # sirina qr koda u pikselima, .rect[3] je visina.
            # print(len(decodedObject))
            # print(decodedObject[0])
            # print(decodedObject[1])
            p1 = decodedObject[0]
            p2 = decodedObject[1]

            points1 = p1.polygon
            pts1 = np.array(points1, np.int32)
            pts1 = pts1.reshape((-1, 1, 2))
            cv2.polylines(im_bgr, [pts1], True, (0, 255, 0), 3)

            points2 = p2.polygon
            pts2 = np.array(points2, np.int32)
            pts2 = pts2.reshape((-1, 1, 2))
            cv2.polylines(im_bgr, [pts2], True, (0, 255, 0), 3)

            j = 0

            for p in pts1:
                # print(p1)
                if (j == 0):
                    x1 = p[0][0]
                    y1 = p[0][1]
                if (j == 2):
                    x2 = p[0][0]
                    y2 = p[0][1]
                j = j + 1
            cv2.circle(im_bgr, (int((x1 + x2) / 2), int((y1 + y2) / 2)), 5, (0, 0, 255), 3)  # calc. the center of the rectangle
            j = 0
            for p in pts2:
                # print(p1)
                if (j == 0):
                    x12 = p[0][0]
                    y12 = p[0][1]
                if (j == 2):
                    x22 = p[0][0]
                    y22 = p[0][1]
                j = j + 1
            cv2.circle(im_bgr, (int((x12 + x22) / 2), int((y12 + y22) / 2)), 5, (0, 0, 255), 3)  # calc. the center of the rectangle

            self.sredinaX = int (( int( (x1 + x2) / 2) + int((x12 + x22) / 2) ) / 2 )
            self.sredinaY = int((int((y1 + y2) / 2) + int((y12 + y22) / 2)) / 2)
            cv2.circle(im_bgr, (self.sredinaX, self.sredinaY), 5, (0, 0, 255), 3)  # sredina arene

        cv2.circle(im_bgr, (428, 240), 5, (255, 0, 0), 3)  # sredina arene
        hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)  

        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask0 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([164, 95, 135])
        upper_red = np.array([197, 186, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        mask_CRVENA = mask0 + mask1

        hsv2 = hsv.copy()
        lower_blue = np.array([86, 120, 63])
        upper_blue = np.array([135, 255, 255])
        mask_PLAVA = cv2.inRange(hsv, lower_blue, upper_blue)

        _, cont, _ = cv2.findContours(mask_CRVENA, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        _, cont2, _ = cv2.findContours(mask_PLAVA, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        if cont:
            cnt = sorted(cont, key=cv2.contourArea, reverse=True)
            if len(cnt) > 1:
                M = cv2.moments(cnt[0])
                cv2.drawContours(im_bgr, [cnt[0]], 0, (0, 255, 0), 3)
                if M["m00"] != 0:
                    self.Xvc = int(M["m10"] / M["m00"])
                    self.Yvc = int(M["m01"] / M["m00"])
                    print('Velika crvena: x =  {}, y = {} '.format(self.Xvc, self.Yvc))
                    cv2.circle(im_bgr, (self.Xvc, self.Yvc), 7, (0, 255, 255), -1)
                    cv2.putText(im_bgr, "Crvena zona", (self.Xvc, self.Yvc), cv2.FONT_HERSHEY_SIMPLEX, 1, (180, 105, 255), 2, cv2.LINE_AA, False)
                M = cv2.moments(cnt[1])
                cv2.drawContours(im_bgr, [cnt[1]], 0, (0, 255, 0), 3)
                if M["m00"] != 0:
                    self.Xmc = int(M["m10"] / M["m00"])
                    self.Ymc = int(M["m01"] / M["m00"])
                    print('Mala crvena: x =  {}, y = {} '.format(self.Xmc, self.Ymc))
                    cv2.circle(im_bgr, (self.Xmc, self.Ymc), 4, (0, 255, 255), -1)
                    cv2.putText(im_bgr, "Crveni sphero", (self.Xmc, self.Ymc), cv2.FONT_HERSHEY_SIMPLEX, 1, (180, 105, 255), 2, cv2.LINE_AA, False)

        if cont2:
            cnt = sorted(cont2, key=cv2.contourArea, reverse=True)
            if len(cnt) > 1:
                M = cv2.moments(cnt[0])
                cv2.drawContours(im_bgr, [cnt[0]], 0, (0, 255, 0), 3)
                if M["m00"] != 0:
                    self.Xvp = int(M["m10"] / M["m00"])
                    self.Yvp = int(M["m01"] / M["m00"])
                    print('Velika plava: x =  {}, y = {} '.format(self.Xvp, self.Yvp))
                    cv2.circle(im_bgr, (self.Xvp, self.Yvp), 7, (0, 255, 255), -1)
                    cv2.putText(im_bgr, "Plava zona", (self.Xvp, self.Yvp), cv2.FONT_HERSHEY_SIMPLEX, 1, (180, 105, 255), 2, cv2.LINE_AA, False)
                M = cv2.moments(cnt[1])
                cv2.drawContours(im_bgr, [cnt[1]], 0, (0, 255, 0), 3)
                if M["m00"] != 0:
                    self.Xmp = int(M["m10"] / M["m00"])
                    self.Ymp = int(M["m01"] / M["m00"])
                    print('Mala plava: x =  {}, y = {} '.format(self.Xmp, self.Ymp))
                    cv2.circle(im_bgr, (self.Xmp, self.Ymp), 4, (0, 255, 255), -1)
                    cv2.putText(im_bgr, "Plavi sphero", (self.Xmp, self.Ymp), cv2.FONT_HERSHEY_SIMPLEX, 1, (180, 105, 255), 2, cv2.LINE_AA, False)

        self.point_R_m.x = self.Xvc / self.pix
        self.point_R_m.y = self.Yvc / self.pix
        self.point_R_s.x = self.Xmc / self.pix
        self.point_R_s.y = self.Ymc / self.pix
        self.point_B_m.x = self.Xvp / self.pix
        self.point_B_m.y = self.Yvp / self.pix
        self.point_B_s.x = self.Xmp / self.pix
        self.point_B_s.y = self.Ymp / self.pix

        # sredina arene
        self.point_QR.x = self.sredinaX / self.pix 
        self.point_QR.y = self.sredinaY / self.pix
        
        show_image(im_bgr)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            print("Vrti se run.\n")
            self.red_s.publish(self.point_R_s)
            self.red_m.publish(self.point_R_m)
            self.blue_s.publish(self.point_B_s)
            self.blue_m.publish(self.point_B_m)
            self.qr_point.publish(self.point_QR)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('blue_zone_node')
    try:
        ps = Position()
        ps.run()
    except rospy.ROSInterruptException: pass
