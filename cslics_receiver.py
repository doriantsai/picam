# /usr/bin/env python3

"""
cslics_receiver.py
Subscribe to cslics_imager topic and accept the image, replay the image/save it as an image file
Author: Dorian Tsai
Email: dorian.tsai@gmail.com
"""

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)    
    rospy.loginfo('Image received...')
    img = 


def receiver():

    rospy.init_node('receiver', anonymous=True)
    rospy.Subscriber('image', String, callback)

    rospy.spin()


if __name__ == '__main__':
    receiver()
