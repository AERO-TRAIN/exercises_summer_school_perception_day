#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import  Point
from cv_bridge import CvBridge
import cv2



class PointSelector:
    def __init__(self, image_topic, point_topic):
        # subcribe to the image topic
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        # create a point publisher
        # Point is a message type that contains 3 float32 values: x, y, z
        self.point_pub = rospy.Publisher(point_topic, Point, queue_size=1)
        

    def image_callback(self, msg):
        #callback function for the image subscriber

    def run(self):
        # main function in which the image is displayed and the mouse callback is set

    def mouse_callback(self, event, x, y, flags, param):
        # mouse callback function that is called when the user clicks on the image


if __name__ == "__main__":
    rospy.init_node("point_selector")

    # set the topic names from the launch file
    image_topic_ = 
    point_topic_ = 

    point_query = PointSelector(image_topic_, point_topic_)
    point_query.run()
    rospy.spin()
    cv2.destroyAllWindows()

        

