import numpy as np
import cv2
from Hitbox import Hitbox
class Hitbox_Detector:
    def __init__(self,color_ranges={'green':(np.array([64, 70, 66]),np.array([77, 207, 134]))}):
        self.color_ranges = color_ranges
        self.colors = [color for color in color_ranges.keys()]

    def draw_hitboxes_on_img(self,img):
        hitboxes = self.detect_hitboxes(img)
        cv2.imshow('hitboxes',self.draw_hitboxes(img,hitboxes))
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def detect_hitboxes(self,img):
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        hitboxes=[]
        for color,color_range in self.color_ranges.items():
            color_lower,color_upper = color_range
            mask = cv2.inRange(hsv, color_lower, color_upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours)>1:
                for contour in contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    if w * h > 10:
                        x,y,w,h = self.adjust_hitbox_ratio(x,y,w,h)
                        hitbox = Hitbox(color,(x, y, x + w, y + h))
                        hitboxes.append(hitbox)
        return hitboxes

    def adjust_hitbox_ratio(self,x,y,w,h):
        new_h = int(4*w/3)
        new_y = int(y - new_h/2)
        return x,new_y,w,new_h
    

    def draw_hitboxes(self,img,hitboxes):
        for hitbox in hitboxes:
            label = hitbox.get_color()
            x1,y1,x2,y2 = hitbox.get_rect()
            cv2.rectangle(img, (x1, y1), (x2, y2), (170,200,200), 1)
        return img



if __name__ == "__main__":
    img = cv2.imread('C:/Users/anshs/csro/src/bot_far.jpg')
    detector = Hitbox_Detector()
    detector.draw_hitboxes_on_img(img)

