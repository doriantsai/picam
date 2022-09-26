#! usr/bin/env python3

# pi camera wrapper object
# configure camera settings
# TODO read configuration file to set relevant picamera configuration parameters
# documentation for PiCamera module: https://picamera.readthedocs.io/en/release-1.13/api_camera.html

import json
import os
from picamera import PiCamera
import datetime
import time
from PIL import Image


class PiCameraWrapper:

    def __init__(self, 
                 camera_index, 
                 resolution,
                 iso,
                 shutter_speed,
                 awb_gains):
        # TODO Python type-setting to ensure valid input types
        # TODO document input units
        # shutterspeed in [ms]
        # iso is image gain, (100-800)
        # init Camera object (wrapper for picam)

        self.camera_index = camera_index
        self.camera = self.get_picamera(resolution)
        PiCamera.CAPTURE_TIMEOUT = 266.0    # double how long we are willing to wait for the camera to capture
        if self.camera.resolution != resolution:
            # apply desired resolution
            self.camera.resolution = resolution
        self.camera.iso = iso
        self.camera.exposure_mode = 'auto'    
        self.camera.shutter_speed = shutter_speed
        
        if awb_gains[0] > 0 and awb_gains[1] > 0:
            self.camera.awb_mode = 'off'
            self.camera.awb_gains = awb_gains


    def rest_auto_adjust(self):
        """
        rest the camera to using auto-exposure and auto white balance.
        Undoes any fixed awb gains or custom auto adjust
        """
        self.camera.iso = 0
        self.camera.shutter_speed = 0
        self.camera.exposure_mode = 'auto'
        self.camera.awb_mode = 'auto'

    
    def set_iso(self, iso=None):
        if iso is None:
            return
        self.camera.iso = iso


    def set_resolution(self, width=None, height=None):
        if width is None or height is None:
            return
        self.camera.resolution = (width, height)

    
    def set_awb_gains(self, red=None, blue=None):
        if red is None and blue is None:
            return
        if red is None:
            red = self.camera.awb_gains[0]
        if blue is None:
            blue = self.camera.awb_gains[1]
        self.camera.awb_mode = 'off'
        self.camera.awb_gains = (red, blue)

    
    def capture_image(self, img_name=None, save_img=False):
        """
        capture and return a single image using PIL
        """
        if img_name is None:
            datestr = datetime.datetime.now()
            img_name = datestr.strftime("%Y%m%d_%H%M%S") + '_img'

        if save_img is True:
            
            self.camera.capture(img_name + '.png', format='png')
            return
        else:
            self.camera.capture(img, format='png')
            img_pil = Image.open(img)
            return img_pil

    
    def get_picamera(self, resolution=None):
        __picamera = PiCamera(resolution=resolution)
        time.sleep(2) # sleep for 2 seconds, camera warm-up time
        return __picamera


    def get_params(self):
        # get picamera settings
        resolution = self.camera.resolution
        cam_params = {
            'sensor_mode': self.camera.sensor_mode,
            'zoom': self.camera.zoom,
            'roi': {
                'x': self.camera.zoom[0],
                'y': self.camera.zoom[1],
                'w': self.camera.zoom[2],
                'h': self.camera.zoom[3],
            },
            'iso': self.camera.iso,
            'exposure_mode': self.camera.exposure_mode,
            'shutter_speed': {
                'nominal': self.camera.shutter_speed,
                'actual': self.camera.exposure_speed
            },
            'awb_mode': self.camera.awb_mode,
            'awb_gains': {
                'red': {
                    'numerator': self.camera.awb_gains[0].numerator,
                    'denominator': self.camera.awb_gains[0].denominator
                },
                'blue': {
                    'numerator': self.camera.awb_gains[1].numerator,
                    'denominator': self.camera.awb_gains[1].denominator
                }
            },
            'resolution': {
                'width': resolution[0],
                'height': resolution[1]
            },
            'analog_gain': {
                'numerator': self.camera.analog_gain.numerator,
                'denominator': self.camera.analog_gain.denominator
            },
            'digital_gain': {
                'numerator': self.camera.digital_gain.numerator,
                'denominator': self.camera.digital_gain.denominator
            },
            'exposure_compensation': self.camera.exposure_compensation,
            'brightness': self.camera.brightness,
            'contrast': self.camera.contrast,
            'saturation': self.camera.saturation,
            'sharpness': self.camera.sharpness,
            'hflip': self.camera.hflip,
            'vflip': self.camera.vflip,
            'rotation': self.camera.rotation,
            'image_denoise': self.camera.image_denoise,
            'revision': self.camera.revision
        }
        return cam_params


if __name__ == "__main__":

    # main function
    # TODO: read config file
    # create camera object
    # set parameters
    # capture an image

    camera_index = 1
    resolution = (2592, 1944)
    iso = 100
    shutter_speed = 10000 # us
    awb_gains = (0.5, 1.5)

    print('setting picamerawrapper')
    PiCam = PiCameraWrapper(camera_index, resolution, iso, shutter_speed, awb_gains)

    # check camera settings:
    param = PiCam.get_params()
    print(param['resolution'])

    print('capturing image')
    PiCam.capture_image(save_img=True)

    print('done')

    import code
    code.interact(local=dict(globals(), **locals()))