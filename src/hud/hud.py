from hud import crosshair, timer, minimap, bottom_hud, kd_info
from csro.msg import GameState, PlayerState

class HUD:
    def __init__(self, player_id):
        self.crosshair = crosshair.Crosshair()
        self.bottom_hud = bottom_hud.BottomHud()
        self.kd_info = kd_info.KDInfo()
        self.timer = timer.Timer()
        self.minimap = minimap.Minimap(player_id)

    def display(self, cv_image, game_state: GameState, player_state: PlayerState):
        self.minimap.display(cv_image)
        self.crosshair.display(cv_image)
        self.bottom_hud.display(cv_image, player_state, game_state.total_hp)
        self.kd_info.display(cv_image, player_state)
        self.timer.display(cv_image, game_state)
        