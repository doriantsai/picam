#! /usr/bin/env python3

"""
read configuration file of camera settings, return dictionary
"""

import json
import os
from pathlib import Path
from collections import namedtuple

# configuration file definition
Config = namedtuple('Config', [
    'camera_index',
    'iso',
    'resolution',
    'red_gain',
    'blue_gain',
    'awb_mode',
    'shutter_speed',
    'exposure_mode'
])


def read_conf(config_file: str = None):
    # read config file from json file
    if config_file is None:
        config_file = Path(__file__).parent / 'config_camera.json'
    else:
        config_file = Path(config_file)
    if config_file.exists():
        with config_file.open('r') as fp:
            conf = json.load(fp)
    else:
        # create a template config file and exit with exception
        from json import dump as json_dump
        with config_file.open('w') as fp:
            json_dump({
                'camera_index': 1,
                'iso': 0,
                'resolution': [2592, 1944],
                'red_gain': -1.0,
                'blue_gain': -1.0,
                'awb_mode': 'auto',
                'shutter_speed': 10000,
                'exposure_mode': 'auto'
            }, fp, indent=4)
        raise FileNotFoundError(f"No config file available, one has been created at '{config_file}'. Please fill it out.")

    # extract configuration file values
    camera_index = int(conf.get('camera_index', -1))
    
    iso = int(conf.get('iso', 0))
    valid_iso_range = {0, 100, 200, 320, 400, 500, 640, 800}
    if iso not in valid_iso_range:
        print(f'warning: iso not in valid iso range: {iso}, default set to 0')
        iso = 0

    resolution_def = (2592, 1944)
    resolution = conf.get('resolution', resolution_def)
    if len(resolution) != 2:
        print(f'warning: resolution length != 2: {resolution}, setting default to {resolution_def}')
        resolution = resolution_def
    
    red_gain = float(conf.get('red_gain', -1.0))
    blue_gain = float(conf.get('blue_gain', -1.0))

    awb_mode = str(conf.get('awb_mode', 'auto'))

    shutter_speed = int(conf.get('shutter_speed', 1000))

    exposure_mode = str(conf.get('exposure_mode', 'auto'))

    # package into namedtuple
    conf_return = Config(
        camera_index = camera_index,
        iso = iso,
        resolution = resolution,
        red_gain = red_gain,
        blue_gain = blue_gain,
        awb_mode = awb_mode,
        shutter_speed = shutter_speed,
        exposure_mode = exposure_mode
    )

    return conf_return


if __name__ == '__main__':
    config = read_conf()
    print('Read configuration: ')
    print(config)