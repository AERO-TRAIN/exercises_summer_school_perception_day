#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

class PublisherListener:
    def __init__(self) -> None:

        # init node
        rospy.init_node("publisher_listener")
        rospy.loginfo_once("[publisher_listener] node init")

        # variables
        self.data_in_sum = 0

        # subscriber
        self.data_in = rospy.Subscriber("/data_in",Int16,callback=self.data_in_cb)

        # pubisher
        self.data_out = rospy.Publisher("/data_out",Int16,queue_size=1)

    def data_in_cb(self,msg):
        """" Callback method """

        rospy.loginfo_once("[publisher_listener] callback init, start sending data..")

        # make sum
        self.data_in_sum += msg.data

        # init msg and publish
        data_out_msg = Int16()
        data_out_msg.data = self.data_in_sum
        
        self.data_out.publish(data_out_msg)


if __name__ == "__main__":
    # init class
    publisher_listener = PublisherListener()

    # sping
    rospy.spin()

        
        