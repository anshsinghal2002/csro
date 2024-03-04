import numpy as np
import cv2

# Read in the image
image = cv2.imread("C:/Users/anshs/csro/opencv_toolkit/ex_img.jpg")

# Convert image to hue-saturation-value from RGB
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define lower and upper limits for different colors
color_ranges = {
    'pink': (np.array([142, 10, 20]), np.array([162, 255, 255])),
    'yellow': (np.array([25, 10, 20]), np.array([45, 255, 255])),
    'blue': (np.array([85, 10, 20]), np.array([97, 255, 255]))
}

# Iterate over color ranges
for color, (lower_limit, upper_limit) in color_ranges.items():
    # Create a mask for the specified color range
    mask = cv2.inRange(hsv_image, lower_limit, upper_limit)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Iterate over all contours for this color
    for contour in contours:
        # Compute the bounding box of the contour
        x, y, w, h = cv2.boundingRect(contour)
        
        if w>10:
            # Draw a bounding box around the contour (color specified)
            if color == 'pink':
                box_color = (147, 20, 255)  # pink color
            elif color == 'yellow':
                box_color = (0, 255, 255)   # yellow color
            elif color == 'blue':
                box_color = (255, 0, 0)     # blue color
            cv2.rectangle(image, (x, y), (x + w, y + h), box_color, 2)

# Display the image with bounding boxes
cv2.imshow('image', image)
cv2.waitKey(0)
