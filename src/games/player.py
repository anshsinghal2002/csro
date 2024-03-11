from csro.msg import PlayerState

class Player:
    def __init__(self, player_id, color_str, total_hp):
        self.id = player_id
        self.color_str = color_str
        self.total_hp = total_hp
        self.hp = total_hp
        self.num_elims = 0
        self.num_respawns = 0

    # Damages the player by "damage" hitpoints
    def hit(self, shooter_player, damage):
        self.hp -= damage
        
        if self.hp <= 0:
            shooter_player.num_elims += 1 
            self.num_respawns += 1
            self.hp = self.total_hp
            return True

        return False

    def getCurrentState(self):
        state = PlayerState()
        state.id = self.id
        state.hp = self.hp
        state.num_elims = self.num_elims
        state.num_respawns = self.num_respawns
        return state
