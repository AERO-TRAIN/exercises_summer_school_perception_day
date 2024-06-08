#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import  Point
from cv_bridge import CvBridge
import cv2



class PointSelector:
    def __init__(self, image_topic, point_topic):
        self.bridge = CvBridge()
        # subcribe to the image topic
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        # create a point publisher
        # Point is a message type that contains 3 float32 values: x, y, z
        
        self.point_pub = rospy.Publisher(point_topic, Point, queue_size=1)

        self.image = None
        self.image_set = False
        self.x_clicked = 0
        self.y_clicked = 0

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.image_set = True

    def run(self):
        while not rospy.is_shutdown():
            if self.image_set:
                clicked_color = [int(self.y_clicked*self.image.shape[0]), int(self.x_clicked*self.image.shape[1])]
                cv2.circle(self.image, (clicked_color[1],clicked_color[0]), 5, (0, 0, 255), -1)
                cv2.imshow("image", self.image)
                cv2.setMouseCallback("image", self.mouse_callback)
                cv2.waitKey(1)
            else:
                rospy.loginfo("Waiting for data")
                rospy.sleep(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.x_clicked = x / self.image.shape[1]
            self.y_clicked = y / self.image.shape[0]
            point = Point()
            point.x = self.x_clicked
            point.y = self.y_clicked
            self.point_pub.publish(point)
            rospy.loginfo("Clicked point: x {} y {}".format(point.x, point.y))


if __name__ == "__main__":
    rospy.init_node("point_selector")

    # read parameters from launch file
    image_topic_ = rospy.get_param("~image_topic", "/camera/color/image")
    point_topic_ = rospy.get_param("~point_topic", "/clicked_point")

    point_query = PointSelector(image_topic_, point_topic_)
    point_query.run()
    rospy.spin()
    cv2.destroyAllWindows()

        

