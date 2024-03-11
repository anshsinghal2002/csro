#!/usr/bin/env python3
import rospy
import cv2
import argparse
from std_msgs.msg import String
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge, CvBridgeError
from hud_elements import crosshair, healthbar, timer, minimap, bottom_hud, kd_info
from game_event_animations import damaged
from hitbox_detector import Hitbox_Detector
from hitbox import Coords
import numpy as np
import random as rand
from csro.srv import RegisterPlayer, GetPlayer, ApplyHit
from csro.msg import GameState, GameEvent
# from csro.srv import RegisterPlayer
# from csro.msg import HitEvent
# from games import PaintballGame 

WINDOW_SIZE_SCALING = 2

class HudUI:
    # instance variables
    def __init__(self, player_id, band_color, camera_upsidedown, get_player, game_state, apply_hit):
        self.image_pub = rospy.Publisher(f"{player_id}/gw_converter_{player_id}_{band_color}",Image,queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(f"/{player_id}/camera/image",Image,self.image_callback)
        self.joy_sub = rospy.Subscriber(f"{player_id}/joy", Joy, self.joy_callback)
        self.game_state_sub = rospy.Subscriber('/game_state', GameState, self.game_state_callback)
        self.game_state = game_state
        self.game_event_sub = rospy.Subscriber(f'/game_event', GameEvent, self.game_event_callback)
        self.apply_hit = apply_hit
        self.cv_image = np.zeros((240,320,3), np.uint8)
        self.hitbox_img = np.zeros((240,320,3), np.uint8)
        self.get_player = get_player

        self.player_id = player_id
        self.band_color = band_color
        self.is_firing = False
        self.camera_upsdwn = camera_upsidedown

        # init hud elements
        self.crosshair = crosshair.crosshair()
        self.bottom_hud = bottom_hud.buttom_hud()
        self.kd_info = kd_info.kd_info()
        self.timer = timer.timer()
        self.minimap = minimap.minimap(player_id)
        self.fire_animation_frame = 0

        # init game event animations
        self.dmg_ani = damaged.damaged()

    def game_state_callback(self, game_state):
        self.game_state = game_state

    def game_event_callback(self, event: GameEvent):
        # TODO: handle different game types
        pass

    def image_callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.hitbox_img = self.cv_image
            
            # Rotate the image by 180 degrees if needed
            if self.camera_upsdwn:
                # Rotate the image by 180 degrees
                self.cv_image = cv2.rotate(self.cv_image, cv2.ROTATE_180)
                self.hitbox_img = cv2.rotate(self.hitbox_img, cv2.ROTATE_180)
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = self.cv_image.shape
        
        self.cv_image = self.game_event_listener(data)

        detector = Hitbox_Detector()
        hitboxes = detector.detect_hitboxes(self.hitbox_img)
        for hitbox in hitboxes:
            label = hitbox.get_color()
            rect = hitbox.get_rect()

            strokeWidth = 1
            color = (170,200,200)
            if hitbox.get_rect().contains(Coords(cols/ 2, rows / 2)):
                strokeWidth = 3
                color = (0,0,255)
            
            cv2.rectangle(self.cv_image, (rect.topLeft.x, rect.topLeft.y), (rect.bottomRight.x, rect.bottomRight.y), color, strokeWidth)

        self.minimap.display(self.cv_image)
        self.crosshair.display(self.cv_image)
        self.bottom_hud.display(self.cv_image,
                                self.get_player(f"{self.band_color}").player,
                                self.game_state.game_state.total_hp)
        self.kd_info.display(self.cv_image, self.get_player(f"{self.band_color}").player)
        self.timer.display(self.cv_image, self.game_state.game_state)
        self.game_event_listener(data)

        if self.is_firing:
            laser_frames = 10
            laser_scaling_factor = 100/laser_frames
            laser_start_point = (int((cols/ 2)+105-((laser_scaling_factor)*self.fire_animation_frame)), int((rows / 2)+105-((laser_scaling_factor)*self.fire_animation_frame)))
            laser_end_point = ((laser_start_point[0]-5),(laser_start_point[1]-5))
            cv2.line(self.cv_image, laser_start_point, laser_end_point, (0, 0, 200), 3)

        resized = cv2.resize(self.cv_image, (int(cols*WINDOW_SIZE_SCALING), int(rows*WINDOW_SIZE_SCALING)))
        cv2.imshow(f"playerID: {self.player_id} | Color: {self.band_color}", resized)
        
        cv2.waitKey(3)
    
    
    def game_event_listener(self, data):
        cv_image = self.cv_image
        
        # # simulate taking damage in game. remove when csro node is done
        # if rand.randint(0, 255) < 10:
        #     cv_image = self.dmg_ani.display(self.cv_image, dead=False)
        
        # simlate death 
        # cv_image = self.dmg_ani.display(self.cv_image, dead=True)    

        return cv_image
    

    def fire(self):
        if self.fire_animation_frame<=20:
            self.fire_animation_frame+=1
        else:
            self.fire_animation_frame=0

    
    def joy_callback(self, data):
        if data.axes[5] < 0:
            self.fire()
            if self.is_firing:
                return

            self.is_firing = True
            
        else:
            self.is_firing = False
        

if  __name__ == '__main__':
 
    register_player = rospy.ServiceProxy('register_player', RegisterPlayer)
    get_player = rospy.ServiceProxy('get_player', GetPlayer)
    apply_hit = rospy.ServiceProxy('apply_hit', ApplyHit)

    parser = argparse.ArgumentParser(description='Fires up HUD elements')
    parser.add_argument('--player_id', type=str, default='default_player', help='player username for CS:RO', action='store')
    parser.add_argument('--band_color', type=str, default='RBY', help="band color that the player's turtlebot is wearing", action='store')
    args, unknown = parser.parse_known_args()

    # then use the get_param method to get the player_id
    plyr_id = rospy.get_param("player_id")
    plyr_clr = rospy.get_param("player_color")
    camera_ori = rospy.get_param("camera_upsidedown")
    try:
        game_state = register_player(plyr_id, plyr_clr)
        game_window = HudUI(plyr_id, plyr_clr, camera_ori, get_player, game_state, apply_hit)
        rospy.init_node(f'game_window_converter_{args.player_id}_{args.band_color}', anonymous=True)
        rospy.Rate(60)
        rospy.spin()
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()