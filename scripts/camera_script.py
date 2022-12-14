#! usr/bin/env python3

from picamera import PiCamera
from PIL import Image
import time

# purpose is to capture an image using a script

print('starting picamera capture')
picam = PiCamera()
picam.iso = 800
picam.sensor_mode=2
picam.resolution = (2592, 1944)
picam.shutter_speed = 10000 # 10 ms
picam.start_preview()

# give camera some time to adjust to conditions
time.sleep(2)
picam.capture('la.jpg')

# try capture a sequence
#print('test capture sequence')
#picam.framerate=30
#picam.capture_sequence(['image%02d.jpg'%(i) for i in range(0,5)])

    
print('all done')

