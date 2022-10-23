#!/usr/bin/env python

import rospy
import cv2
import numpy as np 
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge, CvBridgeError

img = None

class CameraViewer:

    def __init__(self, serial):
        self.bridge = CvBridge()
        self.serial = serial
        self.image_sub = rospy.Subscriber(f"/ximea_ros/ximea_{self.serial}/image_raw", Image, self.callback)
        self.color_pub = rospy.Publisher("/test_color", ColorRGBA, queue_size=10)

    def callback(self,data):
        global img
        try:
          img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)
          
          
        # (rows, cols, channels) = image.shape
        # rows = img.shape[0]
        # cols = img.shape[1]
        # gets all the colours of the middle pixel
        bgr = img[img.shape[0] // 2, img.shape[1] // 2, :]
        #print(f"rows: {img.shape[0]}, cols: {img.shape[1]}")
        
        colour_avg = self.get_colour_average()
        #print(f"avg: {colour_avg}")
        
        # TODO:
        # get average colour of a square 3 pixels around center pixel
        # print(bgr)
        color = ColorRGBA()
        color.r = colour_avg[2]
        color.g = colour_avg[1]
        color.b = colour_avg[0]
        
        viewer.color_pub.publish(color)

    def get_colour_average(self):
        global img
        
        center_r = img.shape[0] // 2
        center_c = img.shape[1] // 2
        
        total_bgr = [0, 0, 0]
        count = 0
        for r_factor in range(-3, 4):
            for c_factor in range(-3, 4):
                count += 1
                for bgr_index in range(0, 3):
                    total_bgr[bgr_index] += img[center_r+r_factor, center_c+c_factor, bgr_index]
                
            
        avg_bgr = np.array(total_bgr) // count
        return avg_bgr

def colour_check_cb(data:ColorRGBA):
    global colour
    colour["r"]=data.r
    colour["g"]=data.g
    colour["b"]=data.b


if __name__ == '__main__':  
    rospy.init_node('image_node', anonymous=True)
    viewer = CameraViewer('31703851')
    
    try:
        while not rospy.is_shutdown():
            if img is not None:
                cv2.imshow("Image", img)   
                cv2.waitKey(1)
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
