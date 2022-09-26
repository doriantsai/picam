#! /usr/bin/env python3

# requires PiCam install
# from PiCameraWrapper import PiCameraWrapper
from PiCameraWrapper import PiCameraWrapper as PCW
import time
import os
import machinevisiontoolbox as mvt
# import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

# config_file = 'config_camera.json'
# PiCam = PCW(config_file=config_file)

# param = PiCam.get_params()
# print(param['shutter_speed'])

# NOTE: probably need to configure resolution to be lower due to raspby
# print('capturing image')
# PiCam.capture_image(save_img=True)
# PiCam.set_shutter_speed(20000)

# print('sleep for 5 sec')
# time.sleep(5) # sleep 5 seconds for settings to take effect

# print('new shutter speed, capture new image')
# param_new = PiCam.get_params()
# print(param_new['shutter_speed'])
# PiCam.capture_image(save_img=True)

# make output folder:
print('create output folder')
out_dir = 'output'
os.makedirs(out_dir, exist_ok=True)

# now, read in images from HQ camera and perform count:
print('get all files in image directory')
img_dir = 'images'
img_list = os.listdir(img_dir)
img_list.sort()

for i, n in enumerate(img_list):
    print(f'{i}: {n}')

# read input image (just the 0th for now)
idx = 0
img_name = os.path.join(img_dir, img_list[idx])
print(f'Investigating image: {img_name}')
img = mvt.Image(img_name)

# NOTE: TODO - resize image, because it's too large for the 2GB pi
# blur image to make thresholding easier and less noisy
# print('smoothing image')
# img = img.smooth(sigma=3)
# img.disp(title='color image')
# img.write(os.path.join(out_dir, os.path.basename(img_name)))

# checkout histogram
print(f'Show histogram of colour channels')
h = mvt.Image.hist(img, nbins=256)
print(h)
figh, axh = img.plothist(h)
figh.savefig(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_rgb_hist.png'))

# hsv colour thresholding
print('colour thresholding by converting into HSV space')
hsv = img.colorspace(conv='HSV')

hsv0 = mvt.Image(hsv.image[:,:,0])

hsv0.disp(title='hsv0 - value')
# hsv0.write(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv0.png'))
# try writing images using matplot lib, as opencv doesn't handle image floats well
plt.imshow(hsv0.image)
plt.savefig(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv0-value.png'))

hsv1 = mvt.Image(hsv.image[:,:,1])
# hsv1.disp(title='hsv1 - saturation')
# hsv1.write(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv1.png'))
plt.imshow(hsv1.image)
plt.savefig(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv1-saturation.png'))

hsv2 = mvt.Image(hsv.image[:,:,2])
# hsv2.disp(title='hsv2 - hue')
# hsv2.write(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv2.png'))
plt.imshow(hsv2.image)
plt.savefig(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv2-hue.png'))

# make hsv histogram
hhsv = mvt.Image.hist(hsv, nbins=256)
fighsv, axhsv = hsv.plothist(hhsv)
fighsv.savefig(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv_hist.png'))


import code
code.interact(local=dict(globals(), **locals()))