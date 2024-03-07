import roslib
import sys
import rospy
import cv2
import argparse
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import os
import asyncio

class damaged:
    def __init__(self) -> None:
        self.damage_color = [0, 0, 255]  # red in BGR
        self.opacity = 0.5
        self.thickness = -1  # colors in the whole rectangle
        pass

    def display(self, cv_image):
        cv_img_cpy = cv_image.copy()  
        cv2.rectangle(cv_img_cpy, 
                        pt1=(0,0), 
                        pt2=(320,240),
                        color=self.damage_color,
                        thickness=self.thickness,
                        )
                # apply opacity effect
        cv_image = cv2.addWeighted(cv_img_cpy,
                                    self.opacity, 
                                    cv_image,  
                                    1-self.opacity, 
                                    0)
                
        return cv_image
        
        pass