#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from light_classification.FRCNNClassifier import FRCNNClassifier
from light_classification.camera_project_classifier import CameraProjectionClassifier
import numpy as np
import cv2
import yaml
import math
import time
import matplotlib.image as mpimg
import light_classification.classify_light as classify_light

STATE_COUNT_THRESHOLD = 1

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        use_inference = rospy.get_param('~use_inference', True)
        rospy.logerr("TL DETECTOR BOOTING UP: use inference (CNN model) = {}".format(use_inference))

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.bridge = None

        self.bridge = CvBridge()

        if use_inference:
            self.light_classifier = FRCNNClassifier(path="./light_classification/")
        else:
            self.light_classifier = CameraProjectionClassifier(self.config)

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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=10**8)

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
        print("TL DETECTOR: IMAGE CB", light_wp, " STATE: ", state)
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

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        self.camera_image.data = cv_image

        pts = self.light_classifier.extract_bounding_box(self.camera_image, light)

        #TODO use light location to zoom in on traffic light in image
        #print('p1 {}, p2 {}, img shape {}'.format(pts[0], pts[3], cv_image.shape))
        #cv2.rectangle(cv_image, (int(pts[0][0]), int(pts[0][1])), (int(pts[3][0]), int(pts[3][1])), (255, 255, 255), 5)
        #mpimg.imsave('tl_detected.png', cv_image, origin='upper')

        return classify_light.classify_light_with_bounding_box(pts, cv_image)

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
        light_pose = Pose()
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
                light_pose.position.x = stop_line_positions[closest_light_index][0]
                light_pose.position.y = stop_line_positions[closest_light_index][1]
            
        if light:
            light_wp_index = self.get_closest_waypoint(light_pose)
            light_wp = self.waypoints.waypoints[light_wp_index]
            state = self.get_light_state(light)
            if light.state == state:
                rospy.loginfo("Traffic Light Predicted CORRECTLY: ")
            else:
                rospy.loginfo("Traffic Light Predicted WRONG!!! ")

            rospy.loginfo("light state {}, predicted {}".format(light.state, state))
            return light_wp_index, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
