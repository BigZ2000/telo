#!/usr/bin/env python3
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from math import sqrt

linear_speed = 0.6
angular_speed = 0.8




class Traker:
    def __init__(self) -> None:
        rospy.loginfo("Subscribe to topic image : /usb_cam/image_raw")
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.image_sub_callback, queue_size=10)
        self.image_pub = rospy.Publisher('/usb_cam/cv_image', Image, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.bridge = CvBridge()

    def image_sub_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.line(cv_image,(0,int(cv_image.shape[0]/2)),(cv_image.shape[1], int(cv_image.shape[0]/2)),(0,255,0),1)

        cv2.line(cv_image,(int(cv_image.shape[1]/2),0),(int(cv_image.shape[1]/2),cv_image.shape[0]),(0,255,0),1)
        center_image  =  (int(cv_image.shape[1]/2), int(cv_image.shape[0]/2))
        cv2.circle(cv_image, center_image, 5, (255, 255, 255), -1)
        gray  = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 3)

        for (x,y,w,h) in faces:
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),5)
            roi_gray  = gray[y:y+h, x:x+w]
            roi_color = cv_image[y:y+h, x:x+w]

            arr = {y:y+h, x:x+w}
            print (arr)
            

            xx = int((x+(x+h))/2)
            yy = int((y+(y+w))/2)

            center = (xx,yy)
            cv2.circle(cv_image, center, 5, (255, 0, 0), -1)

            print("Center of face rectangle is :", center)
            
            cmd = Twist()

            euclidean_distance = int(sqrt((center[0] - center_image[0])**2 + (center[1] - center_image[1])**2))
            euclidean_distance_x = int(sqrt((center[0] - center_image[0])**2))
            euclidean_distance_y = int(sqrt((center[1] - center_image[1])**2))
            if(euclidean_distance > min(w, h)):
                if euclidean_distance_x > min(w, h):
                    print('TELO rotation')
                    if center[0] < center_image[0]:
                        print('move left')
                        cmd.angular.z = angular_speed
                    elif center[0] > center_image[0]:
                        print('move right')
                        cmd.angular.z = -1 * angular_speed
                    else:
                        print('stop')
                elif euclidean_distance_x > min(w, h) and euclidean_distance_x < min(w, h)*2:
                    print('TELO horizontal head servo rotation')
                    if center[0] < center_image[0]:
                        print('servo move left')
                        cmd.angular.y = 0.1
                    elif center[0] > center_image[0]:
                        print('servo move right')
                        cmd.angular.y = -0.1
                    else:
                        print('stop')
                elif euclidean_distance_y > min(w, h):
                    print('TELO vertical head servo rotation')
                    if center[1] < center_image[1]:
                        print('servo move up')
                        cmd.angular.x = -0.1
                    elif center[1] > center_image[1]:
                        print('servo move down')
                        cmd.angular.x = 0.1
                    else:
                        print('stop')
            else:
                print("height:{0}, width:{1}".format(w, h))
                #print("size: ", w*h)
                print("percentage face/image: ", ((w*h)/(cv_image.shape[0]*cv_image.shape[1]))*100)
                print("percentage height_face/image: ", (h/cv_image.shape[0])*100)
                height_face_image = h/cv_image.shape[0] # proportion of the face per the image

                if height_face_image < 0.25:
                    print('move forward')
                    cmd.linear.x = linear_speed
                elif height_face_image > 0.35:
                    print('move backward')
                    cmd.linear.x = -1 * linear_speed
                else:
                    print('stop')
            self.cmd_pub.publish(cmd)
            break
        

        cv_image_reverse =cv2.flip(cv_image,-1)
        cv2.imshow('cv_image',cv_image)
    
        cv2.waitKey(3)

        try:
            rospy.loginfo("Publish opencv image to topic image : /usb_cam/cv_image")
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
      
      
if __name__ == '__main__':
    rospy.loginfo("TELO tracking Node")
    rospy.init_node('telo_traker_node')
    traker = Traker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
        cv2.destroyAllWindows()

