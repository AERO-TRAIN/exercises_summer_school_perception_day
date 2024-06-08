#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class LaptopCameraRos:
    def __init__(self):
        rospy.init_node("laptop_camera_ros")

        # Load camera
        self.cap = cv2.VideoCapture(0)
        
        self.cv_bridge = CvBridge()

        # publisher
        self.publisher = rospy.Publisher("/laptop_camera/image_raw", Image, queue_size=1)

        # timer to call camera handler 
        self.timer = rospy.Timer(rospy.Duration(0.1), self.camera_handler)  

        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        """Release on shutdown"""
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

    def camera_handler(self, event):
        """Method to handle camera image"""
        ret, frame = self.cap.read()

        if not ret:
            rospy.logwarn("Failed to capture image from camera")
            return

        # Convert the frame to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        cv2.waitKey(1)  

        # Publish 
        self.camera_publisher(frame)

    def camera_publisher(self, frame):
        """Method to publish camera frame"""
        image_msg = self.cv_bridge.cv2_to_imgmsg(frame, "rgb8")
        image_msg.header.frame_id = "laptop_camera_link"
        image_msg.header.stamp = rospy.Time.now()
        
        self.publisher.publish(image_msg)

if __name__ == '__main__':
    laptop_camera_ros = LaptopCameraRos()

    rospy.spin()
