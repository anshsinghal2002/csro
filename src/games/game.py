from csro.msg import GameState
from player_node import Player
import rospy

# Represents the base class for a game
class Game:
    def __init__(self, total_hp, game_duration):
        self.players = []
        self.total_hp = total_hp
        self.game_duration = game_duration
        self.game_start_time = None

    # Function to construct a Player from a player_id and a color_str
    # Override to use game specific Player subclass
    def create_player(self, player_id, color_str):
        return Player(player_id, color_str, self.get_total_hp())
    
    # Adds a player to the game
    def add_player(self, req):
        self.players.append(self.create_player(req.player_id, req.color_str))


    def start_game(self):
        self.game_start_time = rospy.get_rostime()

    # Callback for a hit event
    # Override to implement game specific logic
    def on_hit(self, event):
        for player in self.players:
            if player.color_str == event.color_str: 
                return player.hit(1)
            
        return None
    
    def getCurrentState(self):
        state = GameState()
        state.total_hp = self.total_hp
        state.game_start_time = self.game_start_time
        state.game_end_time = self.game_start_time + self.game_duration
        state.players = [ player.getCurrentState() for player in self.players ]
        return state

