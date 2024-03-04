import roslib
import sys
import rospy
import cv2
import argparse
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class timer:
    #######################################
    # timer instance variables + settings #
    #######################################
    def __init__(self):
        self.timer_on = True
        self.start_time = time.time()
        self.timer_max = 60  # set max time to a minute for testing
        self.timer_end = self.start_time + self.timer_max
        self.center_pos = (160, 16)  # what point to center the timer background around
        
        ############################################
        # settings for the background of the timer #
        ############################################
        self.bkgrd_visible = True
        self.bkgrd_color = [128, 128, 128]  # gray
        self.bkgrd_opacity = 0.5  # opacity of timer background (0.0 - 1.0)
        self.bkgrd_thickness = -1  # -1 = colors in the whole rectangle
        self.bkgrd_len = 60
        self.bkgrd_height = 20
        # calculate top left and bottom right coords for timer background
        self.bkgrd_topl = (int(self.center_pos[0]-(self.bkgrd_len/2)), int(self.center_pos[1]+(self.bkgrd_height/2)))
        self.bkgrd_btmr = (int(self.center_pos[0]+(self.bkgrd_len/2)), int(self.center_pos[1]-(self.bkgrd_height/2)))

        #######################
        # timer text settings #
        #######################
        self.text_font = cv2.FONT_HERSHEY_DUPLEX
        self.font_scale = 0.75
        self.font_color = [255, 255, 255]  # white
        self.thickness = 1
        # text outline settings
        self.outline_color = [0, 0, 0]  # black
        self.outline_thickness = 2

    def display (self, cv_image):
        if self.timer_on:
            # if self.bkgrd_visible:
                # print("background active")
                # # copy to apply opacity effect later
                # cv_img_cpy = cv_image.copy()  
                # cv2.rectangle(cv_img_cpy, 
                #               pt1=self.bkgrd_topl, 
                #               pt2=self.bkgrd_btmr,
                #               color=self.bkgrd_color,
                #               thickness=self.bkgrd_thickness,
                #               )
                # # apply opacity effect
                # cv_image = cv2.addWeighted(cv_img_cpy, 
                #                            self.bkgrd_opacity, 
                #                            cv_image, 
                #                            1 - self.bkgrd_opacity, 
                #                            0)

            # calculate the time to be displayed
            remaining_time = self.timer_end - time.time()
            if remaining_time <= 0:
                minutes = 0
                seconds = 0
            else:
                minutes = int(remaining_time // 60)
                seconds = int(remaining_time % 60)

            timer_text = f"{minutes}:{str(seconds).zfill(2)}"

            # calculations to center the timer text

            timer_text_size, height_size = cv2.getTextSize(timer_text, 
                                        fontFace=self.text_font,
                                        fontScale=self.font_scale,
                                        thickness=self.thickness)
            # print(timer_text_size)

            # finalize coordinates
            text_center_posx = (self.center_pos[0] -(timer_text_size[0] // 2)) 
            text_center_posy = (self.center_pos[1] +(timer_text_size[1] // 2))
            text_center_pos = (text_center_posx, text_center_posy)
            
            # put the timer on the image
            # the way we give the text outline is just to display the text twice but with
            # the outline text being slightly bigger.
            # display outline text first:
            cv2.putText(cv_image, 
                        text=timer_text, 
                        org=text_center_pos,
                        fontFace=self.text_font, 
                        fontScale=self.font_scale, 
                        color=self.outline_color, 
                        thickness=self.thickness + self.outline_thickness)
            
            # then the actual timer text:
            cv2.putText(cv_image, 
                        text=timer_text, 
                        org=text_center_pos,
                        fontFace=self.text_font, 
                        fontScale=self.font_scale, 
                        color=self.font_color, 
                        thickness=self.thickness)
        
        pass
