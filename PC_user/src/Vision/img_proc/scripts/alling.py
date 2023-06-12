#!/usr/bin/env python

from __future__ import division
import cv2 as cv
import numpy as np
from sklearn.linear_model import LinearRegression
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class robot:
    def __init__(self):
        self.image = None
        self.sub_img = rospy.Subscriber("/camera/rgb/image_color", Image, self.callback)
        self.pub_vel = rospy.Publisher("/hardware/mobile_base/cmd_vel",Twist,queue_size=1)
        self.slope = 0
    
    def callback(self,data):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        self.image = img
        #self.crop_image()
        self.line_detector()
        #print(slope)
        self.alling()
        cv.imshow("Kinect_image",self.image)
        cv.waitKey(1)

    def crop_image(self):
        img= self.image
        pixels_crop_heigth = 100
        pixels_crop_width = 300
        heigth1 = (img.shape[0]/2) - pixels_crop_heigth
        heigth2 = (img.shape[0]/2) + pixels_crop_heigth
        width1 = (img.shape[1]/2) -  pixels_crop_width
        width2 = (img.shape[1]/2) +  pixels_crop_width 
        crop_img = img[int(heigth1):int(heigth2), int(width1):int(width2)]
        self.image = crop_img

    def line_detector(self):
        average = 1
        detected_lines=[]
        img = self.image
        slopes=[]
        edges = cv.Canny(img,150,150)
        lines = cv.HoughLines(edges, 1, np.pi/180, 200)
        cont = 0
        if lines is not None:
            for line in lines:
                cont = cont + 1
                rho, theta = line[0]
                deg_theta = 90 - np.rad2deg(theta)
                if(abs(deg_theta) < 70) or (abs(deg_theta) > 110):
                    print("ANGulo", deg_theta)
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))
                    slope = (y2-y1)/(x2-x1) if (x2-x1)!=0 else 0
                    if slope < 0.2 and slope > -0.2:
                        slopes.append(slope)
                        detected_lines.append([x1,y1,x2,y2,slope])
                        cv.line(img, (x1, y1), (x2, y2), (0, 0, 255), 4)
        else:
            print("No line detected") 
        if len(slopes) != 0:
            average = sum(slopes)/len(slopes)     
        self.slope = average
    
    def alling(self):
        error = self.slope
        Kp = -6.0
        Kp_m = 6.0
        vel = Twist()
        print(error)
        if abs(error) > 0.02:
            if (error < 0) and (error != 1) :
                vel.angular.z = Kp_m*abs(error)
                error = self.slope
                print("Derechaaaaa")
            elif error > 0 and (error != 1) :
                vel.angular.z = Kp*abs(error)
                error = self.slope
                print("------------------------------------------Izquierdaaaa")
        else: #error == 0:
            vel.linear.x = 0
            vel.linear.y = 0
            vel.angular.z = 0
            print("Lined up")

        self.pub_vel.publish(vel)
        rospy.sleep(0.05)
            
        return True
        
rospy.init_node("Alling")
rate = rospy.Rate(0.1)
robot = robot()
rospy.spin()