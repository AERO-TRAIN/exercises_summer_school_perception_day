#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool

class CircleDetector:
    def __init__(self) -> None:
        # init node
        rospy.init_node("circle_detector")

        # rosparams
        color_topic_name    = rospy.get_param("~color_topic", "/color_topic")
        self.dp_param            = rospy.get_param("~dp","/dp")
        self.min_radius_param    = rospy.get_param("~min_radius","min_radius")
        self.max_radius_param    = rospy.get_param("~max_radius","max_radius")
        self.min_dist_param      = rospy.get_param("~min_dist","min_dist")

        # self.depth_image = None
        self.color_image = None

        self.circles = None
        self.cv_bridge = CvBridge()

        self.latest_center_circle = np.array([0,0])

        # subscribers
        # self.sub_depth = rospy.Subscriber("/d400/aligned_depth_to_color/image_raw",Image,callback=self.depthCallback)
        self.sub_color = rospy.Subscriber(color_topic_name,Image,callback=self.colorCallback)

        # publishers
        self.pub_image  = rospy.Publisher("/camera/detected_circles",Image,queue_size=1)
        self.pub_center = rospy.Publisher("/camera/center",Int16MultiArray,queue_size=1)
        self.pub_detection = rospy.Publisher("/camera/dected",Bool,queue_size=1)

    # def depthCallback(self,msg):
    #     """ Depth callback method: take depth image and convert to cv2"""
        
    #     rospy.loginfo_once("[circle_detector] depth callback init")
        
    #     self.depth_image = self.cv_bridge.imgmsg_to_cv2(
    #             msg, desired_encoding="passthrough"
    #         )

    def colorCallback(self,msg):
        """Color callback method: take color image and convert to cv2 """
        
        rospy.loginfo_once("[circle_detector] color callback init")
        
        self.color_image = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )
        # self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)
        
        # detect circle and publish
        self.detector(msg)

        # publish latest detected center
        self.publish_center_pos()

    def publish_center_pos(self):
        """ Method to publish latest center detected"""
        
        center_msg = Int16MultiArray()

        center_msg.data = self.latest_center_circle

        self.pub_center.publish(center_msg)        

    def detector(self,msg):
        """Detect circle from image and publish"""

        # blur and gray to improve detector
        modified_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)
        modified_image = cv2.GaussianBlur(modified_image, (9, 9), 2)

        circles = cv2.HoughCircles(
            modified_image, cv2.HOUGH_GRADIENT_ALT, dp=self.dp_param, minDist=self.min_dist_param,
            param1=250, param2=0.97, minRadius=self.min_radius_param, maxRadius=self.max_radius_param
        )

        image_msg = Image()
        image_msg.header.stamp = rospy.Time.now()

        if circles is not None and len(circles) > 0:
            rospy.loginfo("[circle_detector] Circles detected")
            circles = np.uint16(np.around(circles))

            # Draw only the first detected circle
            c = circles[0, 0]
            cv2.circle(self.color_image, (c[0], c[1]), c[2], (0, 255, 0), 3)
            cv2.circle(self.color_image, (c[0], c[1]), 1, (0, 0, 255), 5)

            # Publish center pixel
            self.latest_center_circle = [c[0],c[1]] 

        else:
            rospy.loginfo("[circle_detector] No circles detected")

        # Convert back and publish
        image_msg = self.cv_bridge.cv2_to_imgmsg(self.color_image, "bgr8")
        image_msg.header.stamp = rospy.Time.now()
        image_msg.header.frame_id = msg.header.frame_id 

        self.pub_image.publish(image_msg)



if __name__ == '__main__':
    circle_detector = CircleDetector()

    rospy.spin()