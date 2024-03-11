from games.game import Game
from rospy import Duration

class PaintballGame(Game):
    def __init__(self):
        PAINTBALL_TOTAL_HP = 10
        PAINTBALL_GAME_DURATION = Duration(5 * 60)
        super().__init__(PAINTBALL_TOTAL_HP, PAINTBALL_GAME_DURATION)


