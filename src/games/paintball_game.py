from games.player import Player
from game import Game
from rospy import Duration

class PaintballGame(Game):
    def __init__(self):
        super().__init__(total_hp=10, game_duration=Duration(5 * 60))


