import roslib
import sys
import rospy
import cv2
import argparse
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class crosshair:
    ##############################
    # crosshair default settings #
    ##############################
    def __init__(self):
        self.visible = True
        self.pos = (160, 120)  # center of the screen for the turtlebots
        self.color = [0, 255, 0]  # green 
        self.size = 30
        self.thickness = 1


    def display(self, cv_image):
        if self.visible:
            line_size = int(self.size/3)
            half_distance = int(self.size/2)

            # draw cross
            cv2.drawMarker(cv_image, 
                           self.pos, 
                           color=self.color, 
                           thickness=self.thickness, 
                           markerType=cv2.MARKER_CROSS, 
                           markerSize=line_size)
            
            # define corner coordinates
            btm_l = ((self.pos[0]-half_distance),(self.pos[1]-half_distance)) # bottom left
            top_l = ((self.pos[0]-half_distance),(self.pos[1]+half_distance)) # top left
            btm_r = ((self.pos[0]+half_distance),(self.pos[1]-half_distance)) # bottom right
            top_r = ((self.pos[0]+half_distance),(self.pos[1]+half_distance)) # top right

            # draw square around the crosshair
            cv2.rectangle(cv_image, 
                          top_l, 
                          btm_r, 
                          color=self.color, 
                          thickness=self.thickness)
                             

            
            
        pass