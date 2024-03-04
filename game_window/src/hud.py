#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2
import argparse
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
from hud_elements import crosshair, healthbar, timer, minimap
import asyncio

class hud_ui:
    # instance variables
    def __init__(self, player_id, band_color):
        self.image_pub = rospy.Publisher(f"gw_converter_{player_id}_{band_color}",Image,queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image",Image,self.callback)
        
        self.player_id = player_id
        self.band_color = band_color

        self.win_size_scaling = 2  # setting to scale the camera image for the game

        # init hud elements
        self.crosshair = crosshair.crosshair()
        self.healthbar = healthbar.healthbar()
        self.timer = timer.timer()
        self.minimap = minimap.minimap()


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # Let's rotate the image by 180 degrees
        # LMAO LET'S NOT
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        # rows: 240
        # col: 320
        # channels: 3
        # cv2.circle(img=cv_image, center=(160,120), radius=30, color=(0,0,255))

        self.crosshair.display(cv_image)
        self.healthbar.display(cv_image)
        self.timer.display(cv_image)

        self.minimap.display(cv_image)

        cv2.imshow(f"playerID: {self.player_id}", cv2.resize(cv_image, 
                                                             (int(cols*self.win_size_scaling), int(rows*self.win_size_scaling))))
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
    
    
"""
# mini map settings:
 global options: fixed frame
 center around odom
 topic: /scan
 size(m): 0.05
 decay time: 0.2
"""


if  __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Fires up HUD elements')
    parser.add_argument('--player_id', type=str, default='default_player', help='player username for CS:RO', action='store')
    parser.add_argument('--band_color', type=str, default='RBY', help="band color that the player's turtlebot is wearing", action='store')
    args, unknown = parser.parse_known_args()

    game_window = hud_ui(args.player_id, args.band_color)
    print(args)
    rospy.init_node(f'game_window_converter_{args.player_id}_{args.band_color}', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()