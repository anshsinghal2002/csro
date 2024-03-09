#!/usr/bin/env python3

import rospy
from csro.srv import RegisterPlayer, GetPlayer
from csro.msg import HitEvent
from games.paintball_game import PaintballGame

GAME_START_EVENT = "game_start"

class CSROCore:
    def __init__(self):
        # Players list
        # Stores all RegisterPlayer objects
        self.game = PaintballGame()


    def register_player(self, req):
        self.game.add_player(req)
        print(req)
        return 1

    def on_hit(self, event):
        is_elim = self.game.on_hit(event)
        # TODO: send response to shooter and send message to player who got hit (if there was one)

    def get_player(self, req):
        return self.game.get_player_id_from_color_str(req.color_str)


if  __name__ == '__main__':
    core = CSROCore()

    rospy.init_node('csro_core', anonymous=True)
    rospy.Service('register_player', RegisterPlayer, core.register_player)
    rospy.Service('get_player', GetPlayer, core.get_player)

    # Possibily change to request
    rospy.Subscriber('hit', HitEvent, core.on_hit)

    try:
        print("-=-=- csro_core started -=-=-")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

