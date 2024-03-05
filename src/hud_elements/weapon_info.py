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
        self.wpn_text_font = cv2.FONT_HERSHEY_DUPLEX
        self.wpn_name_txt_scaling = 0.4
        self.ammo_txt_scaling = 1
        self.rem_ammo_txt_scaling = 0.5
        self.font_color = [255, 255, 255]  # white
        self.txt_outline_color = [0, 0, 0]  # black
        self.outline_thickness = 1


    def display (self, cv_image):
        # display weapon name
        cv2.putText(cv_image, 
                    text=self.weapon_name, 
                    org=(self.brdr_topl[0], self.brdr_topl[1]-25),
                    fontFace=self.wpn_text_font, 
                    fontScale=self.wpn_name_txt_scaling, 
                    color=self.txt_outline_color, 
                    thickness=self.outline_thickness + 2)
        cv2.putText(cv_image, 
                    text=self.weapon_name, 
                    org=(self.brdr_topl[0], self.brdr_topl[1]-25),
                    fontFace=self.wpn_text_font, 
                    fontScale=self.wpn_name_txt_scaling, 
                    color=self.font_color, 
                    thickness=self.outline_thickness)
        
        # display the ammo 

        # first properly format it so that it is center
        ammo_txt_size , height_size = cv2.getTextSize(f"{self.current_ammo}", 
                                        fontFace=self.wpn_text_font,
                                        fontScale=self.ammo_txt_scaling,
                                        thickness=self.outline_thickness)
        
        text_center_posx = (self.center_pos[0] -(ammo_txt_size[0] // 2)) 
        text_center_posy = (self.center_pos[1] +(ammo_txt_size[1] // 2))
        # print(ammo_txt_size[0])
        text_center_pos = (text_center_posx, text_center_posy - 35)
        cv2.putText(cv_image, 
                    text=f"{self.current_ammo}", 
                    org=text_center_pos,
                    fontFace=self.wpn_text_font, 
                    fontScale=self.ammo_txt_scaling, 
                    color=self.txt_outline_color, 
                    thickness=self.outline_thickness + 2)
        cv2.putText(cv_image, 
                    text=f"{self.current_ammo}", 
                    org=text_center_pos,
                    fontFace=self.wpn_text_font, 
                    fontScale=self.ammo_txt_scaling, 
                    color=self.font_color, 
                    thickness=self.outline_thickness)
        
        # display remaining ammo in inventory
        cv2.putText(cv_image, 
                    text=f"/{self.inventory_remaining_ammo}", 
                    org=(self.center_pos[0] + 25, self.center_pos[1]-25),
                    fontFace=self.wpn_text_font, 
                    fontScale=self.rem_ammo_txt_scaling, 
                    color=self.txt_outline_color, 
                    thickness=self.outline_thickness + 2)
        cv2.putText(cv_image, 
                    text=f"/{self.inventory_remaining_ammo}", 
                    org=(self.center_pos[0] + 25, self.center_pos[1]-25),
                    fontFace=self.wpn_text_font, 
                    fontScale=self.rem_ammo_txt_scaling, 
                    color=self.font_color, 
                    thickness=self.outline_thickness)
        pass
        
