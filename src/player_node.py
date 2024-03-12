#!/usr/bin/env python3
import rospy
import cv2
import argparse
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge, CvBridgeError
from hud.hud import HUD
from game_event_animations import damaged
from hitbox_detector import HitboxDetector
from hitbox import Coords
import numpy as np
from csro.srv import RegisterPlayer, GetPlayer, ApplyHit
from csro.msg import GameState, GameEvent
from csro_core_node import GAME_EVENT_GOT_HIT, GAME_EVENT_ELIMED

WINDOW_SIZE_SCALING = 2

class PlayerNode:
    # instance variables
    def __init__(self, player_id, band_color, camera_upsidedown, get_player, game_state: GameState, apply_hit):
        # Arguments
        self.player_id = player_id
        self.band_color = band_color
        self.camera_upsdwn = camera_upsidedown
        
        # Publishers
        self.image_pub = rospy.Publisher(f"{player_id}/gw_converter_{player_id}_{band_color}", Image, queue_size=10)
        
        # Subscribers
        self.image_sub = rospy.Subscriber(f"/{player_id}/cv_camera/image_raw", Image, self.image_callback)
        self.joy_sub = rospy.Subscriber(f"{player_id}/joy", Joy, self.joy_callback)
        self.game_state_sub = rospy.Subscriber('/game_state', GameState, self.game_state_callback)
        self.game_event_sub = rospy.Subscriber(f'/game_event', GameEvent, self.game_event_callback)
        
        # Service proxies
        self.apply_hit = apply_hit
        self.get_player = get_player
        
        # State
        self.game_state = game_state
        self.is_firing = False
        self.game_event = GameEvent()
        
        # Images
        self.bridge = CvBridge()
        self.cv_image = np.zeros((240,320,3), np.uint8)
        self.hitbox_img = np.zeros((240,320,3), np.uint8)

        # HUD elements & Animations
        self.hud = HUD(player_id)
        self.dmg_ani = damaged.Damaged()
        self.fire_animation_frame = 0
    
    # Callback when the game state changes
    def game_state_callback(self, game_state):
        self.game_state = game_state

    # Callback when this player recieves a GameEvent
    def game_event_callback(self, event: GameEvent):
        self.game_event = event

     # Callback for joy events
    def joy_callback(self, data):
        if data.axes[5] < 0:
            self.play_shoot_anim()
            if self.is_firing:
                return

            self.is_firing = True
            self.fire()
        else:
            self.is_firing = False

    # Progresses the shoot animation forward
    def play_shoot_anim(self):
        if self.fire_animation_frame<=20:
            self.fire_animation_frame+=1
        else:
            self.fire_animation_frame=0

    # Fires a paintball at the crosshair's current location
    def fire(self):
        detector = HitboxDetector()
        hitboxes = detector.detect_hitboxes(self.hitbox_img)

        (rows,cols,channels) = self.cv_image.shape
        for hitbox in hitboxes:
            hit_color_str = hitbox.get_color()
            if hitbox.get_rect().contains(Coords(cols/ 2, rows / 2)):
                is_elim = self.apply_hit(self.player_id, hit_color_str)

    # Callback for every frame
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
        player_state =  self.get_player(f"{self.band_color}").player
        
        # Draw all elements on the image
        self.display_game_event(self.game_event)
        self.display_enemy_hitboxes(cols, rows)
        self.hud.display(self.cv_image, self.game_state, player_state)
        self.display_fire_animation(cols, rows)
        self.display_waiting_screen(cols, rows)
        
        # Show the window
        resized = cv2.resize(self.cv_image, (int(cols*WINDOW_SIZE_SCALING), int(rows*WINDOW_SIZE_SCALING)))
        cv2.imshow(f"playerID: {self.player_id} | Color: {self.band_color}", resized)
        cv2.waitKey(3)
    
    # Displays animations for GameEvents
    def display_game_event(self, game_event):
        cv_image = self.cv_image
            
        if game_event.type == GAME_EVENT_GOT_HIT:
            cv_image = self.dmg_ani.display(self.cv_image, dead=False)
        elif game_event.type == GAME_EVENT_ELIMED:
            cv_image = self.dmg_ani.display(self.cv_image, dead=True)    

        return cv_image
    
    # Displays hitboxes of enemies
    def display_enemy_hitboxes(self, cols, rows):
        detector = HitboxDetector()
        hitboxes = detector.detect_hitboxes(self.hitbox_img)
        for hitbox in hitboxes:
            label = f"{hitbox.get_color()}:{self.get_player(str(hitbox.get_color())).player.hp} hp"
            rect = hitbox.get_rect()

            strokeWidth = 1
            color = (170,200,200)
            if hitbox.get_rect().contains(Coords(cols/ 2, rows / 2)):
                strokeWidth = 3
                color = (0,0,255)
            

            font = cv2.FONT_HERSHEY_SIMPLEX
            label_position = (rect.topLeft.x, rect.topLeft.y - 10) 
            cv2.putText(self.cv_image, label, label_position, font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.rectangle(self.cv_image, (rect.topLeft.x, rect.topLeft.y), (rect.bottomRight.x, rect.bottomRight.y), color, strokeWidth)

    # Displays the fire animation
    def display_fire_animation(self, cols, rows):
        if self.is_firing:
            laser_frames = 20
            laser_scaling_factor = 100/laser_frames
            laser_start_point = (int((cols/ 2)+105-((laser_scaling_factor)*self.fire_animation_frame)), int((rows / 2)+105-((laser_scaling_factor)*self.fire_animation_frame)))
            laser_end_point = ((laser_start_point[0]-5),(laser_start_point[1]-5))
            cv2.line(self.cv_image, laser_start_point, laser_end_point, (0, 0, 200), 3)

    # Displays a screen signifying that the game has not started
    def display_waiting_screen(self, cols, rows):
        if not self.game_state.has_started or self.game_state.game_start_time > self.game_state.game_end_time:
            cv2.rectangle(self.cv_image, (0, 0), (cols, rows), (0,0,0), -1)
            cv2.putText(self.cv_image, "WAITING FOR GAME", (10,rows//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(self.cv_image, "START", (cols//2-50,rows//2+50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)


if  __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Fires up HUD elements')
    parser.add_argument('--player_id', type=str, default='default_player', help='player username for CS:RO', action='store')
    parser.add_argument('--band_color', type=str, default='RBY', help="band color that the player's turtlebot is wearing", action='store')
    args, unknown = parser.parse_known_args()
    rospy.init_node(f'game_window_converter_{args.player_id}_{args.band_color}', anonymous=True)

    rospy.wait_for_service('register_player')
    rospy.wait_for_service('get_player')
    rospy.wait_for_service('apply_hit')
 
    register_player = rospy.ServiceProxy('register_player', RegisterPlayer)
    get_player = rospy.ServiceProxy('get_player', GetPlayer)
    apply_hit = rospy.ServiceProxy('apply_hit', ApplyHit)

    plyr_id = rospy.get_param("player_id")
    plyr_clr = rospy.get_param("player_color")
    camera_ori = rospy.get_param("camera_upsidedown")

    try:
        resp = register_player(plyr_id, plyr_clr)
        node = PlayerNode(plyr_id, plyr_clr, camera_ori, get_player, resp.game_state, apply_hit)
        rospy.Rate(60)
        rospy.spin()
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()