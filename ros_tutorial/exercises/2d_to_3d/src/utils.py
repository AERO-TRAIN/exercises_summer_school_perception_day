import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointField
from sensor_msgs.msg import PointCloud2 
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


def from_two_vectors(vec1, vec2):
    # Returns a quaternion representing a rotation between the two arbitrary vectors a and b. In other words, the built rotation represent a rotation sending the line of direction a to the line of direction b, both lines passing through the origin.
    # it is the same function from eigen but in python https://eigen.tuxfamily.org/dox-devel/Quaternion_8h_source.html

    # The function from_two_vectors(vec1, vec2) in Python is designed to calculate the quaternion that represents the rotation from one vector to another. Here's a breakdown of what each line does:
    
    # vec1 = vec1 / np.linalg.norm(vec1) and vec2 = vec2 / np.linalg.norm(vec2): These two lines normalize the input vectors vec1 and vec2. 
    # Normalizing a vector scales it to have a length of 1, but it keeps its direction. This is done using the np.linalg.norm() function, which calculates the Euclidean norm (also known as the L2 norm or the length) of the vector.
    # cross_prod = np.cross(vec1, vec2): This line calculates the cross product of the two normalized vectors. The cross product of two vectors results in a third vector that is orthogonal (at a right angle) to the input vectors. 
    # The direction of the resulting vector follows the right-hand rule.

    # dot_prod = np.dot(vec1, vec2): This line calculates the dot product of the two normalized vectors. The dot product gives a scalar value that represents the cosine of the angle between the two vectors.
    # s = np.sqrt((1 + dot_prod) * 2): This line calculates a scalar value s that will be used in the quaternion calculation. The scalar s is the square root of twice the sum of 1 and the dot product.

    # invs = 1 / s: This line calculates the inverse of s, which will be used to scale the components of the quaternion.
    # return R.from_quat([cross_prod[0] * invs, cross_prod[1] * invs, cross_prod[2] * invs, s * 0.5]): This line returns a quaternion that represents the rotation from vec1 to vec2. The quaternion is created from the scaled cross product (which forms the vector part of the quaternion) and half of s (which forms the scalar part of the quaternion).

    vec1 = vec1 / np.linalg.norm(vec1)
    vec2 = vec2 / np.linalg.norm(vec2)
    cross_prod = np.cross(vec1, vec2)
    dot_prod = np.dot(vec1, vec2)

    s = np.sqrt((1 + dot_prod) * 2)
    invs = 1 / s
    
    return R.from_quat([cross_prod[0] * invs, cross_prod[1] * invs, cross_prod[2] * invs, s * 0.5])

def  open3d_to_pc2( open3d_cloud, frame_id=None, stamp=None):
        # translate a function from open3d to a ros PointCloud2 message
        # from https://github.com/marcoesposito1988/open3d_conversions/blob/master/src/open3d_conversions/open3d_conversions.py
        # The data structure of each point in ros PointCloud2: 16 bits = x + y + z
        FIELDS_XYZ = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        header = Header()
        if stamp is not None:
            header.stamp = stamp
        if frame_id is not None:
            header.frame_id = frame_id

        o3d_asarray = np.asarray(open3d_cloud.points)

        o3d_x = o3d_asarray[:, 0]
        o3d_y = o3d_asarray[:, 1]
        o3d_z = o3d_asarray[:, 2]

        cloud_data = np.core.records.fromarrays([o3d_x, o3d_y, o3d_z], names='x,y,z')

        if not open3d_cloud.colors:  # XYZ only
            fields = FIELDS_XYZ

        return pc2.create_cloud(header, fields, cloud_data)

def get_pose_stamped(point, normal, frame_id, stamp, translated=0, opposite_direction=False):
    # Create a PoseStamped message with the given point, translated along the normal and
    # oriented to the normal or opposite direction

    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x = point[0] + translated*normal[0]
    pose.pose.position.y = point[1] + translated*normal[1]
    pose.pose.position.z = point[2] + translated*normal[2]

    # convert the normal to a quaternion
    vector_orientation = np.array([1, 0, 0])
    if opposite_direction:
        vector_orientation = np.array([-1, 0, 0])


    next_orientation = from_two_vectors(vector_orientation, normal)
    
    pose.pose.orientation.x = next_orientation.as_quat()[0]
    pose.pose.orientation.y = next_orientation.as_quat()[1]
    pose.pose.orientation.z = next_orientation.as_quat()[2]
    pose.pose.orientation.w = next_orientation.as_quat()[3]

    return pose