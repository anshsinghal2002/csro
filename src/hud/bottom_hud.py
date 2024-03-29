from hud import healthbar, weapon_info

# bottom hud elements are the healthbar and weapons info
# weapons info is displayed above the healthbar which is why
# it is initialized with respect to the location of the healthbar
class BottomHud:
    def __init__(self):
        self.center_pos = (160, 225)  # where to center the healthbar around
        self.healthbar = healthbar.Healthbar(self.center_pos)
        hb_top_center = (self.center_pos[0], self.healthbar.brdr_topl[1])

        # init weapon info to be displayed above the healthbar 
        self.weapon_info = weapon_info.WeaponInfo(self.healthbar.brdr_topl, hb_top_center)
    
    def display(self, cv_image, player_state, total_hp):
        self.healthbar.display(cv_image, player_state, total_hp)
        self.weapon_info.display(cv_image)
        pass
