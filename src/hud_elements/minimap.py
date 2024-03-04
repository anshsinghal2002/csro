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
import matplotlib.pyplot as plt
import os
import asyncio
class minimap:
    def __init__(self) -> None:
        self.visible = True
        # self.minimap_node = rospy.init_node('minimap', anonymous=True)
        rospy.Subscriber('scan', LaserScan, self.laserReadingsCallback)
        self.angles = np.radians(np.arange(360))
        self.laserReadings = np.zeros(360)
        self.map_range = 4  # defines how big the minimap is
        self.map_pt_thickness = 5  # defines how thick the minimap readings look
        self.map_pt_color = [0, 255, 0]  # Green in RGB
        self.map_bkgrnd = [255, 255, 255]  # black map backgorund color
        # self.curr_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        self.curr_path = os.path.dirname(__file__)
        pass
    
    def display(self, cv_image):
        if self.visible and self.laserReadings.size != 0:
        # using the lidar sensor readings, use trig to reconstruct a map of
        # the turtlebot's surroundings
            xcoords = self.laserReadings * np.cos(self.angles)
            ycoords = self.laserReadings * np.sin(self.angles)

            plt.figure(figsize=(self.map_range, self.map_range))
            plt.scatter(xcoords,
                        ycoords,
                        s=self.map_pt_thickness,
                        c="g")
            plt.scatter(0,
                        0,
                        s=self.map_pt_thickness + 20,
                        c="r")
            
            plt.xlim(-self.map_range, self.map_range)
            plt.ylim(-self.map_range, self.map_range)
            plt.axis('off')
            # print(self.curr_path)
            plt.savefig(self.curr_path + '/minimap.png')
            plt.close()
        pass

    def laserReadingsCallback(self, data:LaserScan):
        self.laserReadings = np.array(data.ranges)
        pass