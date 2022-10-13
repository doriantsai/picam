#! /usr/bin/env python3

""""
test code to count circles in image
"""


import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv

img_name = 'images/pi_macro_lens/red_microspheres_12cm.jpg'

# read image
img = cv.imread(img_name)

# make copy of the original image
img_circ = img.copy()

# convert to grayscale
img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# blur image
img_gray = cv.medianBlur(img_gray, 5)

# find circles using Hough transform
circles = cv.HoughCircles(image=img_gray,
                          method=cv.HOUGH_GRADIENT,
                          dp=0.9,
                          minDist=80,
                          param1=110,
                          param2=39,
                          maxRadius=200)

# draw detected circles onto image
for circ, i in enumerate(circles[0,:], start=1):
    # draw outer circle in green
    cv.circle(img_circ, (i[0], i[1]), i[2], (0, 255, 0), 5)
    # draw centre of circle in blue
    cv.circle(img_circ, (i[0], i[1]), 2, (255, 0, 0), 8)

print(f'Number of circles detected: {circ}' )

# convert image to rgb (for matplotlib)
img_circ = cv.cvtColor(img_circ, cv.COLOR_BGR2RGB)

plt.imshow(img_circ)
plt.show()