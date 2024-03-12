from games.game import Game
from rospy import Duration

PAINTBALL_TOTAL_HP = 10
PAINTBALL_GAME_DURATION = Duration(5 * 60)

# A class for a paintball game mode
class PaintballGame(Game):
    def __init__(self):
        super().__init__(PAINTBALL_TOTAL_HP, PAINTBALL_GAME_DURATION)


