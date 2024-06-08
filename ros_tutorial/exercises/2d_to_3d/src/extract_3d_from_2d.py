#!/usr/bin/env python
import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud2, PointField

from cv_bridge import CvBridge
import cv2

from camera_info import CameraInfoHandler

# import the function from the utils.py file
from utils import from_two_vectors, open3d_to_pc2, get_pose_stamped




# Class that receives the image, depth image, camera info and depth camera info and extracts the 3D point and normal of the clicked point which is selected by the user
class Extract3DFrom2D:
    def __init__(self, point_topic, depth_topic,  depth_info_topic, point_pub_topic, extracted_cloud_topic, 
                 crop_w=120, crop_h=120, translation=0.0, opposite_direction=False):
        
        # initialize the CvBridge
        self.bridge = CvBridge()
        
        # initialize the subscribers, image and depth image and the depth camera info
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback)
        self.depth_info_sub = rospy.Subscriber(depth_info_topic, CameraInfo, self.depth_info_callback)
        self.point_sub = rospy.Subscriber(point_topic, Point, self.point_callback)

        # initialize the publishers for the point and the point cloud
        self.retrieved_point_pub = rospy.Publisher(point_pub_topic, PoseStamped, queue_size=1)
        self.cloud_pub = rospy.Publisher(extracted_cloud_topic, PointCloud2, queue_size=1)

        # initialize the image and depth image
        self.image = None
        self.depth_image = None
        self.frame_id = None

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

        self.pcd = o3d.geometry.PointCloud()

    def point_callback(self, msg):
        self.x_clicked = msg.x
        self.y_clicked = msg.y
        self.point_set = True

        point, normal = self.get_3d_point(self.x_clicked, self.y_clicked)
        self.publish_3d_point(point, normal)

    def depth_callback(self, msg):
        #self.depth_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Convert to meters from mm because the depth image is in mm encoding 16UC1
        self.depth_image = self.depth_image * 0.001
        self.depth_image_set = True

    def depth_info_callback(self, msg):
        # set the depth camera info
        if not self.depth_camera_info.is_initialized():
            self.depth_camera_info.set_camera_info(msg)


    # This function given the x and y position of the mouse extract the 3D point and patch around that point, which is then used to compute the normal of the point which is later returned
    def get_3d_point(self, x, y):
        if self.point_set and self.depth_image_set and self.depth_camera_info.is_initialized():
    
            # scale the prompt to the image size of the depth image
            x = int(x * self.depth_image.shape[1])
            y = int(y * self.depth_image.shape[0])

            # compute the boundaries of the crop to stay within the image range            
            y_start = max(0, y - self.crop_h)
            y_end = min(self.depth_image.shape[0], y + self.crop_h)
            x_start = max(0, x - self.crop_w)
            x_end = min(self.depth_image.shape[1], x + self.crop_w)

            # get the center of the image from the camera info
            cx = self.depth_camera_info.get_cx()
            cy = self.depth_camera_info.get_cy()

            # extract spatial information of the clicked point
            clicked_point = np.array([0.0, 0.0, 0.0])
            middle_y = int((y_start+y_end)/2.0)
            middle_x = int((x_start+x_end)/2.0)
            
            # get the clicked point in 3D
            clicked_point[2] =  self.depth_image[middle_y,middle_x] 
            clicked_point[0] = (middle_x - cx) * clicked_point[2] /self.depth_camera_info.get_fx()
            clicked_point[1] = (middle_y - cy) * clicked_point[2] /self.depth_camera_info.get_fy()
            

            # extract a patch of the depth image and convert it to a point cloud
            for i in range(y_start, y_end):
                for j in range(x_start, x_end):
                    if self.depth_image[i,j] != 0 and not np.isnan(self.depth_image[i,j]):

                        point = np.array([0.0, 0.0, 0.0])
                        point[2] = self.depth_image[i,j]
                        point[0] = (j - cx) * point[2] * self.depth_camera_info.get_x_const()
                        point[1] = (i - cy) * point[2] * self.depth_camera_info.get_y_const()
                        
                        self.pcd.points.extend([point])

            if self.pcd.is_empty():
                return None

            # clean the point pcdcloud from nan perform voxel downsampling and radius outlier removal 
            self.pcd = self.pcd.voxel_down_sample(voxel_size=0.03)
            self.pcd, ind = self.pcd.remove_radius_outlier(nb_points=8, radius=3)

            # compute the normals of the point cloud
            self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.8, max_nn=120))
            self.pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
            
            # visualize the point cloud with the normals            
            #o3d.visualization.draw_geometries([self.pcd], point_show_normal=True)
            
            # Create a search KDTree for the point cloud
            pcd_tree = o3d.geometry.KDTreeFlann(self.pcd)
            # Search for the closest point to the clicked point
            [k, ind, _] = pcd_tree.search_knn_vector_3d(clicked_point, 1)
            # save the point and the normal
            point = self.pcd.points[ind[0]]
            normal = self.pcd.normals[ind[0]] 

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
            pose = get_pose_stamped(point, normal, self.depth_camera_info.get_frame_id(), rospy.Time.now(), translated=self.translation, opposite_direction=self.opposite_direction)
            self.retrieved_point_pub.publish(pose)

            # Publish the point cloud
            point_cloud_msg = open3d_to_pc2(self.pcd, frame_id=self.depth_camera_info.get_frame_id(), stamp=rospy.Time.now())
            self.cloud_pub.publish(point_cloud_msg)

            # clear the point cloud for the next iteration
            self.pcd.clear()

            rospy.loginfo("3D point: {}, Normal: {}".format(point, normal))
        else:
            rospy.loginfo("3D point not available")

if __name__ == "__main__":
    rospy.init_node("extract_3d_from_2d")

    # read parameters from launch file
    point_topic_ = rospy.get_param("~point_topic", "/clicked_point")
    depth_topic_ = rospy.get_param("~depth_topic", "/camera/depth/image")
    depth_info_topic_ = rospy.get_param("~depth_info_topic", "/camera/depth/camera_info")
    point_pub_topic_ = rospy.get_param("~point_pub_topic", "/retrieved_point")
    extracted_cloud_topic_ = rospy.get_param("~extracted_cloud_topic", "/point_cloud")

    crop_w_ = rospy.get_param("~crop_w", 60)
    crop_h_ = rospy.get_param("~crop_h", 60)
    translation_ = rospy.get_param("~translation", 0.0)
    opposite_direction_ = rospy.get_param("~opposite_direction", False)  

    print("opposite_direction: ", opposite_direction_)
    print("translation: ", translation_)
    node_pose_extraction = Extract3DFrom2D(point_topic_, depth_topic_, depth_info_topic_, 
                           point_pub_topic_, extracted_cloud_topic_, crop_w_, crop_h_, translation=translation_, opposite_direction=opposite_direction_)

    node_pose_extraction.run()
    rospy.spin()
    cv2.destroyAllWindows()

        