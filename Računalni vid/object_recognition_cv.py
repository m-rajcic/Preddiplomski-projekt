#!/usr/bin/env python2.7
# Import ROS libraries and messages
import rospy
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Bool
# ++++++++++++++++++++++++++++++++++++++++++++++++++
import numpy as np
import cv2.aruco as aruco
import cv2
import math
# +++++++++++++++++++++++++++++++++++++++++++++++++
import sys
import time
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(1)


class Position(object):

    def __init__(self):
        ##print('Inicijalizacija klase')
        self.red_s = rospy.Publisher("/red_s", Point, queue_size=1)
        self.red_m = rospy.Publisher("/red_m", Point, queue_size=1)
        self.blue_s = rospy.Publisher("/blue_s", Point, queue_size=1)
        self.blue_m = rospy.Publisher("/blue_m", Point, queue_size=1)
        self.qr_point = rospy.Publisher("/qr_point", Point, queue_size=1)
        self.centar_slike = rospy.Publisher("/centar_slike", Point, queue_size=1)
        self.qr_finder = rospy.Publisher("/qr_exist", Bool, queue_size=1)
        self.yaw = rospy.Publisher("/yaw", Point, queue_size=1)
        self.pose = rospy.Publisher("/drone_pose", Pose, queue_size=1)
        # objekti
        self.point_R_m = Point()
        self.point_R_s = Point()
        self.point_B_m = Point()
        self.point_B_s = Point()
        self.point_QR = Point()
        self.centar = Point()
        self.qr_find = Bool()
        self.proc_img = Image()
        self.yaw_measured = Point()
        self.current_pose = Pose()
        # inicijalizacija varijabli
        self.yaw_measured.x = 0
        self.yaw2 = 0
        self.point_R_m.x = 0
        self.point_R_m.y = 0
        self.point_R_s.x = 0
        self.point_R_s.y = 0
        self.point_B_m.x = 0
        self.point_B_m.y = 0
        self.point_B_s.x = 0
        self.point_B_s.y = 0
        self.Xvc = 0
        self.Yvc = 0
        self.Xmc = 0
        self.Ymc = 0
        self.Xvp = 0
        self.Yvp = 0
        self.Xmp = 0
        self.Ymvp = 0
        self.point_QR.x = 0
        self.point_QR.y = 0
        self.centar.x = 0
        self.centar.y = 0
        # qr kod
        self.sredinaX = 0
        self.sredinaY = 0
        self.yaw_camera = 0


        # NAMJESTI PRIJE TESTIRANJA NA NEKU VRIJEDNOST
        self.pix = 1.0
        self.qr_find = False
        rospy.Subscriber("/bebop/image_raw", Image, self.image_callback, queue_size=1, buff_size=2 ** 30)

    def image_callback(self, img_msg):
        ##print("Callback")
        now1 = rospy.get_rostime()

        rospy.loginfo(img_msg.header)

        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        im_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        # dimension = im_bgr.shape
        gray = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)
        
        show_image(im_bgr)
        
        lower_red = np.array([25, 60, 60])
        upper_red = np.array([83, 255, 255])
        mask_CRVENA = cv2.inRange(hsv, lower_red, upper_red)

        #lower_red = np.array([164, 95, 135])
        #upper_red = np.array([197, 186, 255])
        #mask1 = cv2.inRange(hsv, lower_red, upper_red)

        # mask_CRVENA = mask0 + mask1

        hsv2 = hsv.copy()
        lower_blue = np.array([86, 120, 63])
        upper_blue = np.array([135, 255, 255])
        mask_PLAVA = cv2.inRange(hsv, lower_blue, upper_blue)

        _, cont, _ = cv2.findContours(mask_CRVENA, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        _, cont2, _ = cv2.findContours(mask_PLAVA, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        if cont:
            cnt = sorted(cont, key=cv2.contourArea, reverse=True)
            # povrsina iz crvenog 
            if len(cnt) > 1:
                cv2.drawContours(im_bgr, [cnt[1]], 0, (0, 255, 0), 3)
                if (cv2.contourArea(cnt[0]) / self.pix ** 2) > 0.02:
                    M = cv2.moments(cnt[0])
                    cv2.drawContours(im_bgr, [cnt[0]], 0, (0, 255, 0), 3)
                    if M["m00"] != 0:
                        self.Xvc = int(M["m10"] / M["m00"])
                        self.Yvc = int(M["m01"] / M["m00"])
                        # trazenje omjer pix i metra
                        # print('Velika crvena: x =  {}, y = {} '.format(self.Xvc, self.Yvc))
                        # cv2.circle(im_bgr, (self.Xvc, self.Yvc), 7, (0, 255, 255), -1)
                        # cv2.putText(im_bgr, "Crvena zona", (self.Xvc, self.Yvc), cv2.FONT_HERSHEY_SIMPLEX, 1, (180, 105, 255), 2, cv2.LINE_AA, False)
                        # print(cv2.contourArea(cnt[0]) / self.pix**2)
                # if (cv2.contourArea(cnt[1]) / self.pix**2) > 0.002 and (cv2.contourArea(cnt[1]) / self.pix**2) < 0.005:
                M = cv2.moments(cnt[1])
                cv2.drawContours(im_bgr, [cnt[1]], 0, (0, 255, 0), 3)
                if M["m00"] != 0:
                    self.Xmc = int(M["m10"] / M["m00"])
                    self.Ymc = int(M["m01"] / M["m00"])
                    ##print('Mala crvena: x =  {}, y = {} '.format(self.Xmc, self.Ymc))
                    # cv2.circle(im_bgr, (self.Xmc, self.Ymc), 4, (0, 255, 255), -1)
                    cv2.putText(im_bgr, "Crveni sphero", (self.Xmc, self.Ymc), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (180, 105, 255), 2, cv2.LINE_AA, False)
                    # print(cv2.contourArea(cnt[1]) / self.pix**2)

        if cont2:
            cnt = sorted(cont2, key=cv2.contourArea, reverse=True)
            if len(cnt) > 1:
                cv2.drawContours(im_bgr, [cnt[1]], 0, (0, 255, 0), 3)
                if (cv2.contourArea(cnt[0]) / self.pix ** 2) > 0.02:
                    M = cv2.moments(cnt[0])
                    cv2.drawContours(im_bgr, [cnt[0]], 0, (0, 255, 0), 3)
                    if M["m00"] != 0:
                        self.Xvp = int(M["m10"] / M["m00"])
                        self.Yvp = int(M["m01"] / M["m00"])
                        # print('Velika plava: x =  {}, y = {} '.format(self.Xvp, self.Yvp))
                        # cv2.circle(im_bgr, (self.Xvp, self.Yvp), 7, (0, 255, 255), -1)
                        # cv2.putText(im_bgr, "Plava zona", (self.Xvp, self.Yvp), cv2.FONT_HERSHEY_SIMPLEX, 1, (180, 105, 255), 2, cv2.LINE_AA, False)
                # if (cv2.contourArea(cnt[1]) / self.pix**2) > 0.002:

                M = cv2.moments(cnt[1])
                cv2.drawContours(im_bgr, [cnt[1]], 0, (0, 255, 0), 3)
                if M["m00"] != 0:
                    self.Xmp = int(M["m10"] / M["m00"])
                    self.Ymp = int(M["m01"] / M["m00"])
                    # print('Mala plava: x =  {}, y = {} '.format(self.Xmp, self.Ymp))
                    # cv2.circle(im_bgr, (self.Xmp, self.Ymp), 4, (0, 255, 255), -1)
                    cv2.putText(im_bgr, "Plavi sphero", (self.Xmp, self.Ymp), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (180, 105, 255), 2, cv2.LINE_AA, False)

        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # treba dodati ako nade kod 

        id_to_find = 72
        marker_size = 10  # - [cm]

        # --- Get the camera calibration path
        camera_matrix = np.array([[396.17782, 0.0, 322.453185], [0.0, 399.798333, 174.243174], [0.0, 0.0, 1.0]])
        camera_distortion = np.array([-0.001983, 0.015844, -0.003171, 0.001506, 0.0])

        # --- 180 deg rotation matrix around the x axis
        R_flip = np.zeros((3, 3), dtype=np.float32)
        R_flip[0, 0] = 1.0
        R_flip[1, 1] = -1.0
        R_flip[2, 2] = -1.0

        # --- Define the aruco dictionary
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()
        font = cv2.FONT_HERSHEY_PLAIN

        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)

        if ids is not None and ids[0] == id_to_find:
            self.qr_find = True 
            # -- ret = [rvec, tvec, ?]
            # -- array of rotation and position of each marker in camera frame
            # -- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
            # -- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

            # -- Unpack the output, get only the first
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
            for p in corners[0]:
                self.sredinaX = (int)(((int)(p[0][0]) + (int)(p[2][0])) / 2)
                self.sredinaY = (int)(((int)(p[0][1]) + (int)(p[2][1])) / 2)
            
            x1 = (int)(p[0][0])
            x2 = (int)(p[2][0])
            y1 = (int)(p[0][1])
            y2 = (int)(p[2][1])
            duljinaDijagonale = math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
            a = duljinaDijagonale * ( math.sqrt(2) / 2 ) 
            self.pix = 5.4 * a
            # povAruco = duljinaDijagonale*duljinaDijagonale / 2

            # -- Draw the detected marker and put a reference frame over it
            aruco.drawDetectedMarkers(im_bgr, corners)
            # aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

            # -- Print the tag position in camera frame
            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f" % (tvec[0], tvec[1], tvec[2])
            # cv2.putText(im_bgr, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # -- Obtain the rotation matrix tag->camera
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T

            # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip * R_tc)

            # -- Print the marker's attitude respect to camera frame
            str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                math.degrees(roll_marker), math.degrees(pitch_marker),
                math.degrees(yaw_marker))
            # cv2.putText(im_bgr, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # -- Now get Position and attitude f the camera respect to the marker
            pos_camera = -R_tc * np.matrix(tvec).T

            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (pos_camera[0], pos_camera[1], pos_camera[2])
            # cv2.putText(im_bgr, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            # -- Get the attitude of the camera respect to the frame
            roll_camera, pitch_camera, self.yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                math.degrees(roll_camera), math.degrees(pitch_camera), # MISLIM DA OVE VRIJEDNOSTI ZELIMO SLAT
                # REGULATORU
                math.degrees(self.yaw_camera))
            # cv2.putText(im_bgr, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.circle(im_bgr, (self.sredinaX, self.sredinaY), 5, (255, 255, 0), 3)
            cv2.circle(im_bgr, (428, 240), 5, (0, 255, 255), 3)
        else:
            self.qr_find = False
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
        self.centar.x = 428.0 / self.pix
        self.centar.y = 240.0 / self.pix

        self.yaw_measured.x = self.yaw_camera

        self.current_pose.position.x = 428.0 / self.pix
        self.current_pose.position.y = 240.0 / self.pix
        self.current_pose.position.z = 0.0
        self.current_pose.orientation.x = 0
        self.current_pose.orientation.y = 0
        self.current_pose.orientation.z = self.yaw_camera
                

        show_image(im_bgr)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # print("Vrti se run.\n")
            self.red_s.publish(self.point_R_s)
            self.red_m.publish(self.point_R_m)
            self.blue_s.publish(self.point_B_s)
            self.blue_m.publish(self.point_B_m)
            self.qr_point.publish(self.point_QR)
            self.centar_slike.publish(self.centar)
            self.qr_finder.publish(self.qr_find)
            self.yaw.publish(self.yaw_measured)
            self.pose.publish(self.current_pose)
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('computer_vision')
    try:
        ps = Position()
        ps.run()
    except rospy.ROSInterruptException:
        pass
