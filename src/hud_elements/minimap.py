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

class minimap:
    def __init__(self) -> None:
        self.visible = True
        matplotlib.use('agg')  # speeds up the matplotlib stuff 
        rospy.Subscriber('scan', LaserScan, self.laserReadingsCallback)
        self.angles = np.radians(np.arange(360))
        self.laserReadings = np.zeros(360)  # init as an array of 0s for now
        self.curr_path = os.path.dirname(__file__)  # directory if we were to display the graph
        
        ####################
        # minimap settings #
        ####################
        self.map_range = 4  # defines how big the minimap is in meters. aka the mini map shows 4 meters around the robot
        self.map_size = 75  # how big the map is in pixels
        self.map_pos = ((320 - self.map_size - 5), 5)  # pos of where the map is
        self.map_pt_thickness = 100  # defines how thick the minimap readings look
        self.map_pt_color = 'g'  # green
        self.turtle_size = self.map_pt_thickness + 100  # size of the turtle on the center of the map
        self.turtle_dot_color = 'b'  # what color the center of the turtle
        self.map_bkgrnd = [255, 255, 255]  # black map backgorund color
        
        pass
    
    def display(self, cv_image):
        if self.visible and self.laserReadings.size != 0:
        # using the lidar sensor readings, use trig to reconstruct a map of
        # the turtlebot's surroundings
            xcoords = self.laserReadings * np.cos(self.angles)
            ycoords = self.laserReadings * np.sin(self.angles)

            plt.figure(figsize=(self.map_range, self.map_range))
            
            # recreate turtle surroundings
            plt.scatter(xcoords,
                        ycoords,
                        s=self.map_pt_thickness,
                        c=self.map_pt_color)
            # plot turtle
            plt.scatter(0,
                        0,
                        s=self.turtle_size,
                        c=self.turtle_dot_color)
            # set axis to extend only as far as the specified range
            plt.xlim(-self.map_range, self.map_range)
            plt.ylim(-self.map_range, self.map_range)
            
            # remove the margins and axis of matplotlib
            plt.axis('off')
            plt.margins(x=0, y=0)

            # convert matplotlib graph into an image that can be displayed with opencv
            fig = plt.gcf()
            fig.canvas.draw()
            map = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
            map = map.reshape(fig.canvas.get_width_height() + (3,))
            # scale down the map image to the right size
            minimap = cv2.resize(map, (self.map_size,self.map_size))
            # rotate the map so that it's orientation matches the turtlebot
            minimap = cv2.rotate(minimap, cv2.ROTATE_90_COUNTERCLOCKWISE)
            # add image onto the game_window by overwriting the pixels
            cv_image[self.map_pos[1]:self.map_pos[1]+minimap.shape[0], self.map_pos[0]:self.map_pos[0]+minimap.shape[1]] = minimap
            # cv2.imshow("Minimap", minimap)
            # plt.savefig(self.curr_path + '/minimap.png', dpi=50, bbox_inches="tight")
            plt.close()
            # return cv_image
        pass
    

    def laserReadingsCallback(self, data:LaserScan):
        self.laserReadings = np.array(data.ranges)
        pass