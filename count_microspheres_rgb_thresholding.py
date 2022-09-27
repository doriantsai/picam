#! /usr/bin/env python3

# requires PiCam install

from PiCameraWrapper import PiCameraWrapper as PCW
import time
import os
import machinevisiontoolbox as mvt
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
# from PIL import Image

# make output folder:
print('create output folder')
out_dir = 'output'
os.makedirs(out_dir, exist_ok=True)

config_file = 'config_camera.json'
PiCam = PCW(config_file=config_file)

params = PiCam.get_params()
print('Camera parameters:')
print(params)

print('capturing image')
img_pil, img_name = PiCam.capture_image(save_dir = out_dir)

# NOTE: can just input PIL image directly to mvt.Image
print(f'Investigating image: {img_name}')
img = mvt.Image(img_name)

# checkout histogram
print(f'Show histogram of colour channels')
h = mvt.Image.hist(img, nbins=256)
print(h)
figh, axh = img.plothist(h)
figh.savefig(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_rgb_hist.png'))

# RGB thresholding (not ideal, but for the sake of demonstration)

b = mvt.Image(img.image[:,:,0])
b.disp(title='img blue channel')
plt.imshow(b.image)
plt.savefig(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_img_blue.png'))
# save as numpy array to preserve image values for thresholding and prevent false-colouring
np.save(os.path.join(out_dir,os.path.basename(img_name)[:-4] + '_img_blue.npy'), b.image)

g = mvt.Image(img.image[:,:,1])
plt.imshow(g.image)
plt.savefig(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_img_green.png'))
np.save(os.path.join(out_dir,os.path.basename(img_name)[:-4] + '_img_green.npy'), g.image)

r = mvt.Image(img.image[:,:,2])
plt.imshow(r.image)
plt.savefig(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_img_red.png'))
np.save(os.path.join(out_dir,os.path.basename(img_name)[:-4] + '_img_red.npy'), r.image)

# plt.show()


# get thresholds via manual inspection!
# (see view_image.py)

# import code
# code.interact(local=dict(globals(), **locals()))

# hue (colour)
R_min = 1
R_max = 250

# saturation (amt of grey)
G_min = 30
G_max = 150

# value (brightness)
B_min = 40
B_max = 120


# do thresholds for each

imt_r = cv.inRange(r.image, R_min, R_max)
imt_g = cv.inRange(g.image, G_min, G_max)
imt_b = cv.inRange(b.image, B_min, B_max)

# display each thresholded image (i.e. each channel)
plt.imsave(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_imt_r.png'), imt_r)
plt.imsave(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_imt_g.png'), imt_g)
plt.imsave(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_imt_b.png'), imt_b)

# all in one threshold:
rgb_min_thresh = (B_min, G_min, R_min)
rgb_max_thresh = (B_max, G_max, R_max)
imt = cv.inRange(img.image, rgb_min_thresh, rgb_max_thresh)
imt = mvt.Image(imt)

plt.imsave(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_imt_rgb.png'), imt.image)

# perform morphological operations to clean up the thresholded image
# try to separate connections


# open to get rid of salt and pepper noise
imt = imt.open(se=np.ones((5,5)))
imt.write(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_imt1_open.png'))

# close to get rid of small interior holes
imt = imt.close(se=np.ones((3,3)))
imt.write(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_imt2_close.png'))

imt = imt.erode(se=np.ones((9,9)))
imt.write(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_imt3_erode.png'))

imt = imt.dilate(se=np.ones((9,9)))
imt.write(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_imt4_dilate.png'))

# now count blobs!
print('blob analysis')

b = mvt.Blob(imt)

# show blobs
imb = b.drawBlobs(imt, None, None, None, contourthickness=-1)
imb.disp('blobs')
b.printBlobs()
imb.write(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_imb.png'))

# done! perhaps there's a better way:
# 1) Hough transforms
# 2) machine learning (which we ultimately need to do for the variation in shapes/sizes of real coral spawn)

import code
code.interact(local=dict(globals(), **locals()))