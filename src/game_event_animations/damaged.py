import cv2

class Damaged:
    def __init__(self) -> None:
        #########################
        # damage flash settings #
        #########################
        # flashes red when you take damage or stays red when you're ded
        self.damage_color = [0, 0, 255]  # red in BGR
        self.opacity = 0.5
        self.thickness = -1  # colors in the whole rectangle

        ######################
        # d message settings #
        ######################
        self.d_message = "You got painted!"
        self.center_pos = (160, 30)

        self.text_font = cv2.FONT_HERSHEY_DUPLEX
        self.font_scale = 0.5
        self.txt_color = [255, 255, 255]  # white
        self.txt_outline_color = [0, 0, 0]  # black
        self.text_colors = [self.txt_outline_color, self.txt_color]
        self.text_thickness = 1
        self.outline_thickness = 2
        self.text_thicknesses = [self.outline_thickness, 0]

        self.txt_size , height_size = cv2.getTextSize(f"{self.d_message}", 
                                        fontFace=self.text_font,
                                        fontScale=self.font_scale,
                                        thickness=self.outline_thickness)

    def display(self, cv_image, dead: bool):
        # copy image to apply opacity effect
        cv_img_cpy = cv_image.copy()  

        # draw the rectangle that covers the whole 320x240 screen
        cv2.rectangle(cv_img_cpy, 
                        pt1=(0,0), 
                        pt2=(320,240),
                        color=self.damage_color,
                        thickness=self.thickness,
                        )
        
        # apply opacity effect
        cv_image = cv2.addWeighted(cv_img_cpy,
                                    self.opacity, 
                                    cv_image,  
                                    1-self.opacity, 
                                    0)
        
        # if dead, display the d_message
        if dead:
            self.display_outlined_text(cv_image,
                                        text=self.d_message,
                                        pos=(self.center_pos[0]-(self.txt_size[0]//2), self.center_pos[1]+(self.txt_size[1]//2)),
                                        scale=self.font_scale,
                                        text_thickness=self.text_thickness)  
                
        return cv_image
        
    
    def display_outlined_text(self, cv_image, text, pos, scale, text_thickness):
        for i in range(2):
            cv2.putText(cv_image, 
                        text=text, 
                        org=pos,
                        fontFace=self.text_font, 
                        fontScale=scale, 
                        color=self.text_colors[i], 
                        thickness=self.text_thicknesses[i] + text_thickness)
