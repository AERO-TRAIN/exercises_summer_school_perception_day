#!/usr/bin/env python
import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud2

from cv_bridge import CvBridge
import cv2

from camera_info import CameraInfoHandler

# import the function from the utils.py file
from utils import  open3d_to_pc2, get_pose_stamped


# Class that receives the image, depth image, camera info and depth camera info and extracts the 3D point and normal of the clicked point which is selected by the user
class Extract3DFrom2D:
    def __init__(self, point_topic, depth_topic,  depth_info_topic, point_pub_topic, extracted_cloud_topic, 
                 crop_w=120, crop_h=120, translation=0.0, opposite_direction=False):
        
        # initialize the CvBridge
        
        # initialize the subscribers, image and depth image and the depth camera info
        self.depth_sub = 
        self.depth_info_sub = 
        self.point_sub = 

        # initialize the publishers for the point and the point cloud
        self.retrieved_point_pub = 
        self.cloud_pub =

        # initialize the image and depth image


        # Initialize the camera info handlers
        self.depth_camera_info = CameraInfoHandler()

        self.depth_image_set = False
        self.point_set = False

        self.x_clicked = 0
        self.y_clicked = 0

        self.crop_w = crop_w
        self.crop_h = crop_h
        self.translation = translation
        self.opposite_direction = opposite_direction

        # pointcloud object in Open3D
        self.pcd = o3d.geometry.PointCloud()

    def point_callback(self, msg):
        # store the clicked point

        # extract the 3D point and normal
        
        # publish the point and the normal


    def depth_callback(self, msg):
        #self.depth_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Convert to meters from mm because the depth image is in mm encoding 16UC1

    def depth_info_callback(self, msg):
        # set the depth camera info


    # This function given the x and y position of the mouse extract the 3D point and patch around that point, which is then used to compute the normal of the point which is later returned
    def get_3d_point(self, x, y):
        if self.point_set and self.depth_image_set and self.depth_camera_info.is_initialized():
    
            # scale the prompt to the image size of the depth image
            

            # compute the boundaries of the crop to stay within the image range            

            # get the center of the image from the camera info

            # extract spatial information of the clicked point

            
            # get the clicked point in 3D

            

            # extract a patch of the depth image and convert it to a point cloud

            # if empty return none    
            
            # clean the point pcdcloud from nan perform voxel downsampling and radius outlier removal 


            # compute the normals of the point cloud
           
            # visualize the point cloud with the normals            
            
            # Create a search KDTree for the point cloud

            # Search for the closest point to the clicked point

            # save the point and the normal

            #return the point and the normal
            return point, normal
        
        return None
    

    def run(self):
        # loop until the node is shutdown
        while not rospy.is_shutdown():
            if self.point_set and self.depth_image_set and self.depth_camera_info.is_initialized():

                # Draw visual circles on the depth image to match the one prompted on the color image
                clicked_depth = [int(self.y_clicked*self.depth_image.shape[0]), int(self.x_clicked*self.depth_image.shape[1])]
                cv2.circle(self.depth_image, (clicked_depth[1],clicked_depth[0]), 5, (0, 0, 255), -1)                
                # draw a rectangle around the clicked point using the clicked_depth
                cv2.rectangle(self.depth_image, (clicked_depth [1] - self.crop_w, clicked_depth[0] - self.crop_h), (clicked_depth [1] + self.crop_w, clicked_depth [0] + self.crop_h), (0, 255, 0), 2)

                # show the depth image
                cv2.imshow("depth", self.depth_image)
                cv2.waitKey(1)
            else:
                rospy.loginfo("Waiting for data")
                rospy.sleep(1)

    def publish_3d_point(self, point, normal): 
        # publish the point and the normal
        if point is not None:
        
            # get pose stamped message

            # publish pose stamped

            # convert the point cloud to a PointCloud2 message and publish it

            # clear the point cloud for the next iteration
            self.pcd.clear()

            rospy.loginfo("3D point: {}, Normal: {}".format(point, normal))
        else:
            rospy.loginfo("3D point not available")

if __name__ == "__main__":
    rospy.init_node("extract_3d_from_2d")

    # read parameters from launch file
    point_topic_ = 
    depth_topic_ = 
    depth_info_topic_ = 
    point_pub_topic_ = 
    extracted_cloud_topic_ = 

    # parameter for the crop size 
    crop_w_ = 
    crop_h_ = 
    # define the translation along the normal 
    translation_ = 
    # define the direction of the normal if it is opposite to the camera or not
    opposite_direction_ = 

    node_pose_extraction = Extract3DFrom2D(point_topic_, depth_topic_, depth_info_topic_, 
                           point_pub_topic_, extracted_cloud_topic_, crop_w_, crop_h_, translation=translation_, opposite_direction=opposite_direction_)

    node_pose_extraction.run()
    rospy.spin()
    cv2.destroyAllWindows()

        