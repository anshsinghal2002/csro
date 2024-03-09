#!/usr/bin/env python3
import rospy
import cv2
import argparse
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from hud_elements import crosshair, healthbar, timer, minimap, bottom_hud, kd_info
from game_event_animations import damaged
import numpy as np
import random as rand
# from csro.srv import RegisterPlayer
# from csro.msg import HitEvent
# from games import PaintballGame 

WINDOW_SIZE_SCALING = 2

class HudUI:
    # instance variables
    def __init__(self, player_id, band_color, camera_upsidedown):
        self.image_pub = rospy.Publisher(f"gw_converter_{player_id}_{band_color}",Image,queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image",Image,self.image_callback)
        self.cv_image = np.zeros((240,320,3), np.uint8)
        self.player_id = player_id
        self.band_color = band_color
        self.camera_upsdwn = camera_upsidedown

        # init hud elements
        self.crosshair = crosshair.crosshair()
        self.bottom_hud = bottom_hud.buttom_hud()
        self.kd_info = kd_info.kd_info()
        self.timer = timer.timer()
        self.minimap = minimap.minimap()

        # init game event animations
        self.dmg_ani = damaged.damaged()


    def image_callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Rotate the image by 180 degrees if needed
            if self.camera_upsdwn:
                self.cv_image = cv2.rotate(self.cv_image, cv2.ROTATE_180)
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = self.cv_image.shape
        
        self.cv_image = self.game_event_listener(data)

        self.minimap.display(self.cv_image)
        self.crosshair.display(self.cv_image)
        self.bottom_hud.display(self.cv_image)
        self.kd_info.display(self.cv_image)
        self.timer.display(self.cv_image)
        self.game_event_listener(data)

        resized = cv2.resize(self.cv_image, (int(cols*WINDOW_SIZE_SCALING), int(rows*WINDOW_SIZE_SCALING)))
        cv2.imshow(f"playerID: {self.player_id} | Color: {self.band_color}", resized)
        
        cv2.waitKey(3)
    
    
    def game_event_listener(self, data):
        cv_image = self.cv_image
        
        # simulate taking damage in game. remove when csro node is done
        if rand.randint(0, 255) < 10:
            cv_image = self.dmg_ani.display(self.cv_image, dead=False)
        
        # simlate death 
        # cv_image = self.dmg_ani.display(self.cv_image, dead=True)    

        return cv_image

if  __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Fires up HUD elements')
    parser.add_argument('--player_id', type=str, default='default_player', help='player username for CS:RO', action='store')
    parser.add_argument('--band_color', type=str, default='RBY', help="band color that the player's turtlebot is wearing", action='store')
    args, unknown = parser.parse_known_args()

    # in order to get the args from the launch file:
    # first find the namespace the node is under after using the launch file
    plyr_ns = rospy.get_namespace()

    # then use the get_param method to get the player_id
    plyr_id = rospy.get_param(f"{plyr_ns}/player_id")
    plyr_clr = rospy.get_param(f"{plyr_ns}/player_color")
    camera_ori = rospy.get_param(f"{plyr_ns}/camera_upsidedown")
    game_window = HudUI(plyr_id, plyr_clr, camera_ori)
    rospy.init_node(f'game_window_converter_{args.player_id}_{args.band_color}', anonymous=True)
    rospy.Rate(60)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()