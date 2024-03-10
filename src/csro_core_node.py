#!/usr/bin/env python3

import rospy
from csro.srv import RegisterPlayer, GetPlayer, ApplyHit, ApplyHitRequest
from csro.msg import GameState, GameEvent
from games.paintball_game import PaintballGame

GAME_START_EVENT = "game_start"
GAME_EVENT_GOT_HIT = "got_hit"
GAME_EVENT_ELIMED = "elimed"


class CSROCore:
    def __init__(self):     
        self.game = PaintballGame()
        self.game_state_pub = rospy.Publisher('/game_state', GameState)

    # RegisterPlayer service callback
    def register_player(self, req):
        self.game.add_player(req)
        print(req)
        return 1
    
    # GetPlayer service callback
    def get_player(self, req):
        return self.game.get_player_from_color_str(req.color_str).getCurrentState()

    # Sends a game event to a specific player
    def publish_player_game_event(self, id, event: GameEvent):
        player_game_event_pub = rospy.Publisher(f'/{id}/game_event', GameEvent)
        player_game_event_pub.publish(event)
        

    # Sends a game event to all players
    def broadcast_game_event(self, event: GameEvent):
        for player in self.game.players:
            self.publish_player_game_event(player.id, event)

    # ApplyHit service callback
    def apply_hit(self, req: ApplyHitRequest):
        is_elim = self.game.on_hit(req)
        
        hit_player = self.get_player_from_color_str(req.hit_color_str)
        event = GameEvent(GAME_EVENT_GOT_HIT)
        if is_elim:
            event = GameEvent(GAME_EVENT_ELIMED)
        
        # Let the hit player know that they've been hit / elimed
        self.publish_player_game_event(hit_player.id, event)
        
        # Broadcast the game state to all players
        self.game_state_pub.publish(self.game.getCurrentState())  

        # Let the shooter know if they elimed the player they hit or not
        return is_elim


if  __name__ == '__main__':
    core = CSROCore()

    rospy.init_node('csro_core', anonymous=True)
    rospy.Service('register_player', RegisterPlayer, core.register_player)
    rospy.Service('get_player', GetPlayer, core.get_player)
    rospy.Service('apply_hit', ApplyHit, core.apply_hit)

    try:
        print("-=-=- CSRO core started -=-=-")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

