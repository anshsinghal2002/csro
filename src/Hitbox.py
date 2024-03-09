# Represents an x y coordinate pair
class Coords:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Represents a rectangle
class Rect:
    def __init__(self, topLeft, bottomRight):
        self.topLeft = topLeft
        self.bottomRight = bottomRight

    def contains(self, coords):
        # If less than top left return false
        if coords.x < self.topLeft.x or coords.y < self.topLeft.y:
            return False
        
        # If more than bottom right return false
        if coords.x > self.bottomRight.x or coords.y > self.bottomRight.y:
            return False
        
        return True

# Represents the hitbox for a player identified by the color of their band
class Hitbox:
    def __init__(self,color,rect):
        self.color = color
        self.rect = rect
    
    # Gets the color of the band detected for this hitbox
    def get_color(self):
        return self.color
    
    # Gets the rect 
    def get_rect(self):
        return self.rect
