#!/usr/bin/env python3

import rospy
from csro.srv import RegisterPlayer
from csro.msg import HitEvent
from games import PaintballGame 

GAME_START_EVENT = "game_start"



class CSROCore:
    def __init__(self, game_event_pub):
        # Players list
        # Stores all RegisterPlayer objects
        self.game = PaintballGame()


    def register_player(self, req):
        self.game.add_player(req)
        return 1

    def on_hit(self, event):
        is_elim = self.game.on_hit(event)
        # TODO: send response to shooter and send message to player who got hit (if there was one)


if  __name__ == '__main__':
    core = CSROCore()

    rospy.init_node('csro_core', anonymous=True)
    s = rospy.Service('register_player', RegisterPlayer, core.register_player)

    # Possibily change to request
    s = rospy.Subscriber('hit', HitEvent, core.on_hit)

    try:
        print("-=-=- csro_core started -=-=-")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

