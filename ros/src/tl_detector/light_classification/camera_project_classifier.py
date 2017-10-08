import rospy
import numpy as np
import math
import tf
from sensor_msgs.msg import CameraInfo
from image_geometry.cameramodels import PinholeCameraModel


class CameraProjectionClassifier:

    def __init__(self, config):
        self.config = config
        self.listener = tf.TransformListener()

    def extract_bounding_box(self, image_msg, light_msg):
        return self.project_to_image_plane(light_msg.pose.pose.position, image_msg.header.stamp)

    def project_to_image_plane(self, point_in_world, timestamp):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world
            timestamp (Ros Time): Ros timestamp

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        camera_info = CameraInfo()

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']

        camera_info.width = self.config['camera_info']['image_width']
        camera_info.height = self.config['camera_info']['image_height']

        #print("fx {}, fy {}".format(fx, fy))

        camera_info.K = np.array([[fx, 0, camera_info.width / 2],
                                  [0, fy, camera_info.height / 2],
                                  [0, 0, 1.]], dtype=np.float32)
        camera_info.P = np.array([[fx, 0, camera_info.width / 2, 0],
                                  [0, fy, camera_info.height / 2, 0],
                                  [0, 0, 1., 0]])
        camera_info.R = np.array([[1., 0, 0],
                                  [0, 1., 0],
                                  [0, 0, 1.]], dtype=np.float32)

        camera = PinholeCameraModel()
        camera.fromCameraInfo(camera_info)

        #print("point_in_world = {}".format(str(point_in_world)))
        #print("camera projection matrix ", camera.P)

        # get transform between pose of camera and world frame
        trans = None
        point_in_camera_space = None
        point_in_image = None
        bbox_points_camera_image = []

        euler_transforms = (
            math.radians(90),  # roll along X to force Y axis 'up'
            math.radians(-90 + -.75),  # pitch along Y to force X axis towards 'right', with slight adjustment for camera's 'yaw'
            math.radians(-9)  # another roll to orient the camera slightly 'upwards', (camera's 'pitch')
        )
        euler_axes = 'sxyx'

        try:
            self.listener.waitForTransform("/base_link",
                  "/world", timestamp, rospy.Duration(0.1))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", timestamp)

            camera_orientation_adj = tf.transformations.quaternion_from_euler(*euler_transforms, axes=euler_axes)

            trans_matrix = self.listener.fromTranslationRotation(trans, rot)
            camera_orientation_adj = self.listener.fromTranslationRotation((0, 0, 0), camera_orientation_adj)

            #print("trans {}, rot {}".format(trans, rot))
            #print("transform matrix {}".format(trans_matrix))

            point = np.array([point_in_world.x, point_in_world.y, point_in_world.z, 1.0])

            # this point should match what you'd see from being inside the vehicle looking straight ahead.
            point_in_camera_space = trans_matrix.dot(point)

            #print("point in camera frame {}".format(point_in_camera_space))

            final_trans_matrix = camera_orientation_adj.dot(trans_matrix)

            # this point is from the view point of the camera (oriented along the camera's rotation quaternion)
            point_in_camera_space = final_trans_matrix.dot(point)

            #print("point in camera frame adj {}".format(point_in_camera_space))

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        bbox_points = [(point_in_camera_space[0] - 0.5, point_in_camera_space[1] - 1.1, point_in_camera_space[2], 1.0),
                       (point_in_camera_space[0] + 0.5, point_in_camera_space[1] + 1.1, point_in_camera_space[2], 1.0),
                       (point_in_camera_space[0] - 0.5, point_in_camera_space[1] - 1.1, point_in_camera_space[2], 1.0),
                       (point_in_camera_space[0] + 0.5, point_in_camera_space[1] + 1.1, point_in_camera_space[2], 1.0)]

        # these points represent the bounding box within the camera's image
        for p in bbox_points:
            bbox_points_camera_image.append(camera.project3dToPixel(p))

        #print("point in image {}".format(bbox_points_camera_image))

        return bbox_points_camera_image
