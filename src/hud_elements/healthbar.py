import cv2
from csro.msg import PlayerState

class healthbar:
    ######################
    # healthbar settings #
    ######################
    def __init__(self, center_pos):
        self.visible = True
        # health bar settings
        self.health_color = [255, 255, 255]  # white in BGR
        self.length = 240
        self.height = 16
        self.center_pos = center_pos  # what point to center the healthbar around
            # calculate top left and bottom right coords for health bar
        self.top_l = (int(self.center_pos[0]-(self.length/2)), int(self.center_pos[1]+(self.height/2)))
        self.btm_r = (int(self.center_pos[0]+(self.length/2)), int(self.center_pos[1]-(self.height/2)))
        self.max_health = 120893
        self.current_health = 120893

        ##############################
        # Health Percentage Settings #
        ##############################
        self.hlth_prct_visible = True
        self.hlth_txt_color = [128, 128, 128]  # gray
        self.hlth_txt_font = cv2.FONT_HERSHEY_DUPLEX
        self.hlth_prct_font_scale = 0.5
        self.hlth_prct_thickness = 1
        self.health_percentage = int(round((self.current_health/self.max_health), 2) * 100)
        

        ###############################
        # health bar boarder settings #
        ###############################
        self.boarder_visible = True
        self.boarder_color = [0, 0, 0]  # black
        self.b_thick = 2
            # calculate boarder top left and bottom right
        self.brdr_topl = ((self.top_l[0]-self.b_thick), (self.top_l[1]+self.b_thick))
        self.brdr_btmr = ((self.btm_r[0]+self.b_thick), (self.btm_r[1]-self.b_thick))
        self.thickness = -1  # -1 = colored in rectangle


    def display(self, cv_image, player_state: PlayerState, total_hp):

        # update health values
        self.max_health = total_hp
        self.current_health = player_state.hp
        self.health_percentage = int(round((self.current_health/self.max_health), 2) * 100)

        if self.visible:
        # draw boarder if turned on
            if self.boarder_visible:
                cv2.rectangle(cv_image, self.brdr_topl, self.brdr_btmr, self.boarder_color, self.thickness)
            
            # calculate the current healthbar length and adjust the bottom right point accordingly
            # bandaid fix: when health is 0, health bar is still visible as a sliver
            if self.current_health >=0:
                cur_health_len = int(self.length*(self.current_health/self.max_health))
                new_btmr = ((self.top_l[0]+cur_health_len), (self.btm_r[1]))

                cv2.rectangle(cv_image, self.top_l, new_btmr, self.health_color, self.thickness)
            
            if self.hlth_prct_visible:
                # center with respect to health bar's center
                hp_str = str(self.health_percentage)
                hlth_text_size, height_size = cv2.getTextSize(hp_str, 
                                        fontFace=self.hlth_txt_font,
                                        fontScale=self.hlth_prct_font_scale,
                                        thickness=self.hlth_prct_thickness)
                text_center_posx = (self.center_pos[0] -(hlth_text_size[0] // 2)) 
                text_center_posy = (self.center_pos[1] +(hlth_text_size[1] // 2))
                text_center_pos = (text_center_posx, text_center_posy)

                cv2.putText(cv_image, 
                        text=hp_str, 
                        org=text_center_pos,
                        fontFace=self.hlth_txt_font, 
                        fontScale=self.hlth_prct_font_scale, 
                        color=self.hlth_txt_color, 
                        thickness=self.hlth_prct_thickness)
                        
        
                
            # # code to test health bar
            # self.current_health -= 1
            # self.health_percentage = int(round((self.current_health/self.max_health), 2) * 100)
        pass
        
            
        
