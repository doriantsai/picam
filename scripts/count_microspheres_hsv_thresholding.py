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

param = PiCam.get_params()
# print(param['shutter_speed'])

# NOTE: probably need to configure resolution to be lower due to raspby
print('capturing image')
img_pil, img_name = PiCam.capture_image(save_dir = out_dir)
# PiCam.set_shutter_speed(20000)

# print('sleep for 5 sec')
# time.sleep(5) # sleep 5 seconds for settings to take effect

# print('new shutter speed, capture new image')
# param_new = PiCam.get_params()
# print(param_new['shutter_speed'])
# PiCam.capture_image(save_img=True)

# now, read in images from HQ camera and perform count:
# print('get all files in image directory')
# img_dir = 'images'
# img_list = os.listdir(img_dir)
# img_list.sort()

# for i, n in enumerate(img_list):
#     print(f'{i}: {n}')

# # read input image (just the 0th for now)
# idx = 0
# img_name = os.path.join(img_dir, img_list[idx])
print(f'Investigating image: {img_name}')
img = mvt.Image(img_name)

# # NOTE: TODO - resize image, because it's too large for the 2GB pi
# # blur image to make thresholding easier and less noisy
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
plt.savefig(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv0-value-plot.png'))
# plt.imsave(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv0-value-image.png'),
        #    hsv0.image)
np.save(os.path.join(out_dir,os.path.basename(img_name)[:-4] + '_hsv0.npy'), hsv0.image)

hsv1 = mvt.Image(hsv.image[:,:,1])
# hsv1.disp(title='hsv1 - saturation')
# hsv1.write(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv1.png'))
plt.imshow(hsv1.image)
plt.savefig(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv1-saturation-plot.png'))
# plt.imsave(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv1-saturation-image.png'),
#            hsv1.image)
np.save(os.path.join(out_dir,os.path.basename(img_name)[:-4] + '_hsv1.npy'), hsv1.image)


hsv2 = mvt.Image(hsv.image[:,:,2])
# hsv2.disp(title='hsv2 - hue')
# hsv2.write(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv2.png'))
plt.imshow(hsv2.image)
plt.savefig(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv2-hue-plot.png'))
# plt.imsave(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv2-hue-image.png'),
#            hsv2.image)
np.save(os.path.join(out_dir,os.path.basename(img_name)[:-4] + '_hsv2.npy'), hsv2.image)



# make hsv histogram
hhsv = mvt.Image.hist(hsv, nbins=256)
fighsv, axhsv = hsv.plothist(hhsv)
fighsv.savefig(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv_hist.png'))

# plt.show()

# get thresholds via manual inspection!
# (see view_image.py)


# hue (colour)
H_min = 345
H_max = 360

# saturation (amt of grey)
S_min = 0.5
S_max = 1.0

# value (brightness)
V_min = 0.5
V_max = 0.99


# do thresholds for each

imt_hue = cv.inRange(hsv0.image, H_min, H_max)
imt_sat = cv.inRange(hsv1.image, S_min, S_max)
imt_val = cv.inRange(hsv2.image, V_min, V_max)

# display each thresholded image (i.e. each channel)
plt.imsave(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hue_threshold.png'), imt_hue)
plt.imsave(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_sat_threshold.png'), imt_sat)
plt.imsave(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_val_threshold.png'), imt_val)

# all in one threshold:
hsv_min_thresh = (H_min, S_min, V_min)
hsv_max_thresh = (H_max, S_max, V_max)
imt = cv.inRange(hsv.image, hsv_min_thresh, hsv_max_thresh)
imt = mvt.Image(imt)

plt.imsave(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_hsv_threshold.png'), imt.image)

# perform morphological operations to clean up the thresholded image
# try to separate connections


# open to get rid of salt and pepper noise
imt = imt.open(se=np.ones((5,5)))
imt.write(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_imt1_open.png'))

# close to get rid of small interior holes
imt = imt.close(se=np.ones((3,3)))
imt.write(os.path.join(out_dir, os.path.basename(img_name)[:-4] + '_imt2_close.png'))

imt = imt.erode(se=np.ones((19,19)))
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

# import code
# code.interact(local=dict(globals(), **locals()))