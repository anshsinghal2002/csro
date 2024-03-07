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

class weapon_info:
    def __init__(self, healthbar_topl, hb_top_center) -> None:
        self.visible = True
        self.weapon_name = "Weapon"
        self.current_ammo = 6
        self.inventory_remaining_ammo = 12


        self.brdr_topl = healthbar_topl
        self.center_pos = hb_top_center
        self.text_font = cv2.FONT_HERSHEY_DUPLEX
        self.wpn_name_txt_scaling = 0.4
        self.ammo_txt_scaling = 1
        self.rem_ammo_txt_scaling = 0.5
        self.font_color = [255, 255, 255]  # white
        self.txt_outline_color = [0, 0, 0]  # black
        self.text_colors = [self.txt_outline_color, self.font_color]
        self.text_thickness = 1
        self.outline_thickness = 2
        self.text_thicknesses = [self.outline_thickness, 0]


    def display (self, cv_image):
        # display weapon name
        self.display_outlined_text(cv_image,
                                   text=self.weapon_name,
                                   pos=(self.brdr_topl[0], self.brdr_topl[1]-25),
                                   scale=self.wpn_name_txt_scaling,
                                   text_thickness=self.text_thickness)
        
        # display the ammo 

        # first properly format it so that it is center
        ammo_txt_size , height_size = cv2.getTextSize(f"{self.current_ammo}", 
                                        fontFace=self.text_font,
                                        fontScale=self.ammo_txt_scaling,
                                        thickness=self.outline_thickness)
        
        text_center_posx = (self.center_pos[0] -(ammo_txt_size[0] // 2)) 
        text_center_posy = (self.center_pos[1] +(ammo_txt_size[1] // 2))
        text_center_pos = (text_center_posx, text_center_posy - 35)
        
        self.display_outlined_text(cv_image,
                                text=f"{self.current_ammo}",
                                pos=text_center_pos,
                                scale=self.ammo_txt_scaling,
                                text_thickness=self.text_thickness)
        
        # display remaining ammo in inventory
        self.display_outlined_text(cv_image,
                                text=f"/{self.inventory_remaining_ammo}",
                                pos=(self.center_pos[0] + 25, self.center_pos[1]-25),
                                scale=self.rem_ammo_txt_scaling,
                                text_thickness=self.text_thickness)
        
        
    def display_outlined_text(self, cv_image, text, pos, scale, text_thickness):
        # this iterates twice over two arrays for color and thicknesses
        # displaying the text two times in order to get the outlined text effect
        for i in range(2):
            cv2.putText(cv_image, 
                        text=text, 
                        org=pos,
                        fontFace=self.text_font, 
                        fontScale=scale, 
                        color=self.text_colors[i], 
                        thickness=self.text_thicknesses[i] + text_thickness)
        pass