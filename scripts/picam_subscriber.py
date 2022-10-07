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

class ImageReceiver(object):
    def __init__(self):
        # params
        self.image = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(1) # Hz
        cv.namedWindow("window", 1)

        # subscriber:
        rospy.Subscriber('/picam/image', Image, self.callback)

    def callback(self, msg):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)    
        rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)

        # show the image
        cv.imshow("window", receiver.image)
        cv.waitKey(2)

    def receive_message(self):
        rospy.init_node('receiver', anonymous=True)
        # should subscribe via init
        rospy.spin()
        cv.destroyAllWindows()


if __name__ == '__main__':
    try:
        
        receiver = ImageReceiver()
        receiver.receive_message()


    except rospy.ROSInterruptException:
        pass