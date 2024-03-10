from csro.msg import GameState
from csro.srv import ApplyHitRequest
from games.player import Player
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
        return Player(player_id, color_str, self.total_hp)
    
    def get_player_from_color_str(self, color_str) -> Player:
        for player in self.players:
            if player.color_str == color_str:
                return player
        
        return None
    
    def get_player_by_id(self, id) -> Player:
        for player in self.players:
            if player.id == id:
                return player
        
        return None
    
    # Adds a player to the game
    def add_player(self, req):
        self.players.append(self.create_player(req.player_id, req.color_str))

    
    def start_game(self):
        self.game_start_time = rospy.get_rostime()

    # Callback for a hit event
    # Override to implement game specific logic
    def apply_hit(self, req: ApplyHitRequest):
        hit_player = self.get_player_from_color_str(req.color_str)
        return hit_player.hit(self.get_player_by_id(req.shooter_id), 1)
    
    def getCurrentState(self):
        state = GameState()
        state.total_hp = self.total_hp
        state.game_start_time = self.game_start_time
        state.game_end_time = self.game_start_time + self.game_duration
        state.players = [ player.getCurrentState() for player in self.players ]
        return state

