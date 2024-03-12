import cv2
from csro.msg import PlayerState

class KDInfo:
    def __init__(self) -> None:
        self.visible = True
        self.k = 420
        self.d = 69

        # where to display it one the screen (top left in this case)
        self.pose = (2,10)

        # text settings
        self.text_font = cv2.FONT_HERSHEY_DUPLEX
        self.txt_scaling = 0.4
        self.font_color = [255, 255, 255]  # white
        self.txt_outline_color = [0, 0, 0]  # black
        self.text_colors = [self.txt_outline_color, self.font_color]
        self.text_thickness = 1
        self.outline_thickness = 2
        self.text_thicknesses = [self.outline_thickness, 0]

        # calculations for formatting. height_size is not needed
        self.txt_size, height_size = cv2.getTextSize(f"{self.k}", 
                                        fontFace=self.text_font,
                                        fontScale=self.txt_scaling,
                                        thickness=self.outline_thickness)
        self.line_spacing = 5
        pass

    
    def display(self, cv_image, player_state: PlayerState):
        # update instance variables
        self.k = player_state.num_elims
        self.d = player_state.num_respawns
        
        if self.visible:
            self.display_outlined_text(cv_image,
                                    text=f"K: {self.k}",
                                    pos=self.pose,
                                    scale=self.txt_scaling,
                                    text_thickness=self.text_thickness)
            self.display_outlined_text(cv_image,
                                    text=f"D: {self.d}",
                                    pos=(self.pose[0], self.pose[1]+self.txt_size[1]+self.line_spacing),
                                    scale=self.txt_scaling,
                                    text_thickness=self.text_thickness)            
        pass



    def display_outlined_text(self, cv_image, text, pos, scale, text_thickness):
        # this iterates twice over two arrays for color and thicknesses
        # displaying the text two times in order to get the outlined text effect
        for i in range(2):
            cv2.putText(cv_image, 
                        text=text, 
                        org=pos,
                        fontFace=self.text_font, 
                        fontScale=scale, 
                        color=self.text_colors[i], 
                        thickness=self.text_thicknesses[i] + text_thickness)