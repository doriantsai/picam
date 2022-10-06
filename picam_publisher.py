#! /usr/bin/env python3

"""
cslics imager/picam publisher
a ROS node that publishes picam status and images from HQ picam at set intervals
author: Dorian Tsai
email: dorian.tsai@gmail.com
"""

# TODO question to Gav: should this be part of the picam repo, or a separate thing?

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from picamera import PiCamera
from PIL import Image
import time
from io import BytesIO
from cv_bridge import CvBridge 

def capture_image(camera):
    stream = BytesIO()
    camera.capture(stream, 'jpeg')
    stream.seek(0)
    image_pil = Image.open(stream)
    return image_pil
    

def imager():
    pub = rospy.Publisher('image', Image, queue_size=10)
    rospy.init_node('picam', anonymous=True)
    # unsure of camera capture rate - need to check, but pertty sure it's slow atm
    rate = rospy.Rate(0.1) # 0.25 Hz

    # setup picamera # TODO load PiCameraWrapper
    picam = PiCamera()
    picam.iso = 800
    picam.sensor_mode = 2
    picam.resolution = (2592, 1944)
    picam.shutter_speed = 100000 # 100 ms
    time.sleep(2) # cam some time to adjust to conditions

    while not rospy.is_shutdown():

        # get image
        # picam.capture('test.jpg')
        rospy.loginfo('Capture image')
        img = capture_image(picam)

        br = CvBridge()
        pub.publish(br.cv2_to_imgmsg(img))
        
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        rate.sleep()



if __name__ == '__main__':
    try:
        imager()
    except rospy.ROSInterruptException:
        pass

