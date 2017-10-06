#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image, CameraInfo
from image_geometry.cameramodels import PinholeCameraModel
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import numpy as np
import matplotlib.image as mpimg
import cv2
import yaml
import math
import time

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        use_inference = rospy.get_param('~use_inference', True)
        print("TL DETECTOR BOOTING UP: use inference = {}".format(use_inference))
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.bridge = None
        self.light_classifier = None
        self.listener = None

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.light_classifier = True #TLClassifier()

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        time.sleep(10)
    

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        # print("TL DETECTOR: IMAGE CB", light_wp, " STATE: ", state)
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if (state == TrafficLight.RED or state == TrafficLight.YELLOW) else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        closest_wp_index = None
        min_distance = 1e+10
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        for index, wp in enumerate(self.waypoints.waypoints):
            dist = dl(pose.position, wp.pose.pose.position)
            if dist < min_distance:
                closest_wp_index = index
                min_distance = dist

        return closest_wp_index

    def project_to_image_plane(self, point_in_world, timestamp):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

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

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        pts = self.project_to_image_plane(light.pose.pose.position, self.camera_image.header.stamp)

        #TODO use light location to zoom in on traffic light in image
        #print('p1 {}, p2 {}, img shape {}'.format(pts[0], pts[3], cv_image.shape))
        #cv2.rectangle(cv_image, (int(pts[0][0]), int(pts[0][1])), (int(pts[3][0]), int(pts[3][1])), (255, 255, 255), 5)
        #mpimg.imsave('tl_detected.png', cv_image, origin='upper')

        pts = np.array(pts, dtype=np.int)
        img_shape = (self.config['camera_info']['image_width'], self.config['camera_info']['image_height'])

        state = TrafficLight.UNKNOWN

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        light_pixels = cv_image[min(max(0, pts[0][1]), img_shape[1]):min(max(0, pts[3][1]), img_shape[1]),
                                min(max(0, pts[0][0]), img_shape[0]):min(max(0, pts[3][0]), img_shape[0]),
                                :]
        if len(light_pixels) > 0:
            light_pixels_r = light_pixels[0]
            state = TrafficLight.RED if len(np.where(light_pixels_r > 220)) else TrafficLight.UNKNOWN

        #Get classification
        return state

    def get_closest_traffic_light(self, light_position):
        if (not self.lights):
            return None

        #print("Trying to find TF light closest to: ", light_position)
        dl = lambda a, b: math.sqrt((a.x-b[0])**2 + (a.y-b[1])**2)
        min_dist = 1e+10
        closest_light = None
        for light in self.lights:
            light_pose = light.pose.pose
            dist = dl(light_pose.position, light_position)
            if dist < min_dist:
                min_dist = dist
                closest_light = light
        
        return closest_light

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose and self.waypoints):
            waypoint_index_closest_to_car_position = self.get_closest_waypoint(self.pose.pose)
            
            current_wp_pose = self.waypoints.waypoints[waypoint_index_closest_to_car_position]
            # print("Closest WP to Car POSE: ", current_wp_pose.pose.pose)
            #TODO find the closest visible traffic light (if one exists)
            buffer_space_in_meters = 50

            min_light_dist = 1e+10
            closest_light_index = None

            dl = lambda a, b: math.sqrt((a.x-b[0])**2 + (a.y-b[1])**2)
            for index, light_position in enumerate(stop_line_positions):
                light_x = light_position[0]
                car_position = current_wp_pose.pose.pose.position
                if (abs(car_position.x-light_x) < buffer_space_in_meters): #and traffic light is facing us.
                    dist = dl(current_wp_pose.pose.pose.position, light_position)
                    if dist < 50 and dist < min_light_dist:
                        #print("Found a close Traffic Light: ", light_position)
                        min_light_dist = dist
                        closest_light_index = index


            if closest_light_index != None: 
                light = self.get_closest_traffic_light(stop_line_positions[closest_light_index])
                #light.pose.pose.position.x = stop_line_positions[closest_light_index][0]
                #light.pose.pose.position.y = stop_line_positions[closest_light_index][1]
            
        if light:
            light_wp_index = self.get_closest_waypoint(light.pose.pose)
            light_wp = self.waypoints.waypoints[light_wp_index]
            if self.light_classifier is not None:
                state = self.get_light_state(light)
                if light.state == state:
                    print("Traffic Light Predicted CORRECTLY: ")
                else:
                    print("Traffic Light Predicted WRONG!!! ")
            #time.sleep(5)
            return light_wp_index, light.state #state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
