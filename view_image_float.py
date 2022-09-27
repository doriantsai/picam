#! /usr/bin/env python3

# view image float using matplotlib
# assume image is saved already (from count_microspheres)

import matplotlib.pyplot as plt
import numpy as np

# img_name = 'output/red_microspheres_microlens_12cmhigh_1_hsv0-value-image.png'
# img_name = 'output/red_microspheres_microlens_12cmhigh_1_hsv0.npy'
# img_name = 'output/red_microspheres_microlens_12cmhigh_1_hsv1.npy'
img_name = 'output/red_microspheres_microlens_12cmhigh_1_hsv2.npy'
img = np.load(img_name)
# img = plt.imread(img_name)

# manually inspect images/pixel value ranges for thresholding, record

plt.matshow(img)
plt.show()