#!/usr/bin/env python3

# Author: Dorian Tsai
# Date: 2020 Sept 17
# Course: EGB320

# Purpose: control picamera image exposure settings using picamera module,
# rather than opencv videocapture (since video4linux2 driver does not support
# white balance control of the picamera)

# following code examples from:
# https://picamera.readthedocs.io/en/release-1.9/recipes1.html#capturing-consistent-images
# https://www.pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv

# init camera and grab reference to raw camera capture
cam = PiCamera()
imWidth = 640
imHeight = 480
cam.resolution = (imWidth, imHeight)  # pixels
cam.framerate = 60  # fps
rawCapture = PiRGBArray(cam, size=(imWidth, imHeight))

# allow camera to warm up
while cam.analog_gain <= 1:
    time.sleep(0.1)

# get/set camera properties
print('analog_gain = ', cam.analog_gain)

# for reasonable values of shutter_speed, use exposure_speed
print('shutter_speed = ', cam.shutter_speed)
print('exposure_speed = ', cam.exposure_speed)
cam.shutter_speed = cam.exposure_speed
# cam.shutter_speed = 16560
print('shutter_speed set = ', cam.shutter_speed)

print('exposure_mode = ', cam.exposure_mode)
cam.exposure_mode = 'off'
print('exposure_mode set = ', cam.exposure_mode)

awbg = cam.awb_gains  # get auto-white balance settings
print('awb_mode = ', cam.awb_mode)
cam.awb_mode = 'off'
print('awb_mode set = ', cam.awb_mode)

# cam.awb_gains = 1.9
print('awb_gain = ', awbg)
cam.awb_gains = awbg
print('awb_gain set = ', cam.awb_gains)

print('iso = ', cam.iso)
cam.iso = 200  # 100-200 reasonable for daylight
print('iso set', cam.iso)

# capture frames from camera
for frame in cam.capture_continuous(rawCapture, format='bgr',
                                    use_video_port=True):
    # format bgr for opencv
    im = frame.array

    # display image/wait for keypress
    cv.imshow('picam frame', im)
    k = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for next frame
    rawCapture.truncate(0)

    if k == ord('q'):
        break
