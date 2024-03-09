import numpy as np
import cv2
from hitbox import Hitbox, Rect, Coords

class ColorRange:
    def __init__(self, lowerBound, upperBound):
        self.lowerBound = np.array(lowerBound)
        self.upperBound = np.array(upperBound)

red_ranges = [ColorRange([0,50,50], [10,255,255]), ColorRange([170,50,50], [180,255,255])]
green_ranges = [ColorRange([64, 70, 66], [77, 207, 134])]

class Hitbox_Detector:
    def __init__(self,color_ranges={'red': red_ranges}):
        self.color_ranges = color_ranges
        self.colors = [color for color in color_ranges.keys()]

    def draw_hitboxes_on_img(self,img):
        hitboxes = self.detect_hitboxes(img)
        cv2.imshow('hitboxes',self.draw_hitboxes(img,hitboxes))

    def detect_hitboxes(self,img):
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        hitboxes=[]
        for color,ranges in self.color_ranges.items():
            mask = None
            
            for color_range in ranges:
                range_mask = cv2.inRange(hsv, color_range.lowerBound, color_range.upperBound)
                if mask is None:
                    mask = range_mask
                else:
                    mask = mask + range_mask

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours)>=1:
                for contour in contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    if w * h > 10:
                        x,y,w,h = self.adjust_hitbox_ratio(x,y,w,h)
                        hitbox = Hitbox(color, Rect(Coords(x, y), Coords(x + w, y + h)))
                        hitboxes.append(hitbox)

        return hitboxes

    def adjust_hitbox_ratio(self,x,y,w,h):
        new_h = int(4*w/3)
        new_y = int(y - new_h/2)
        return x,new_y,w,new_h
    

    def draw_hitboxes(self,img,hitboxes):
        for hitbox in hitboxes:
            label = hitbox.get_color()
            rect = hitbox.get_rect()
            cv2.rectangle(img, (rect.topLeft.x, rect.topLeft.y), (rect.bottomRight.x, rect.bottomRight.y), (170,200,200), 1)
        return img



if __name__ == "__main__":
    img = cv2.imread('C:/Users/anshs/csro/src/bot_far.jpg')
    detector = Hitbox_Detector()
    detector.draw_hitboxes_on_img(img)

