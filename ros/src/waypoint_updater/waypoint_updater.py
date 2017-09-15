#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import tf.transformations
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
MAX_SEARCH_WPS = 100

class WaypointUpdater(object):
    all_waypoints = None
    last_closest_wp_index = None
    last_pose = None

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=10)

        # TODO: Add other member variables you need below
        self.loop()

    # angle calc formula from: https://gamedev.stackexchange.com/questions/4467/comparing-angles-and-working-out-the-difference
    def angle_diff(self,a,b):
        return math.pi - abs(abs(a-b) - math.pi)

    def get_waypoint_heading_diff(self, waypoint, position, orientation):
        car_heading = self.get_car_heading(orientation)
        theta = math.atan2(waypoint.pose.pose.position.y - position.y,
                           waypoint.pose.pose.position.x - position.x)
        return self.angle_diff(theta,car_heading)

    def get_closest_waypoint_index(self, position, orientation):
        heading = self.get_car_heading(orientation)
        min_distance = 1000
        min_i = None

        range_for_search = range(len(self.all_waypoints))
        optimized = False
        if (self.last_closest_wp_index):
            optimized = True
            range_for_search = range(self.last_closest_wp_index, self.last_closest_wp_index+MAX_SEARCH_WPS)

        for i in range_for_search:
            i = i % len(self.all_waypoints)
            waypoint = self.all_waypoints[i]
            distance = self.p2p_distance(position, waypoint.pose.pose.position)

            if (distance <= min_distance):
                min_distance = distance
                min_i = i

        min_heading_diff = self.get_waypoint_heading_diff(self.all_waypoints[min_i], position, orientation)
        next_i = (min_i+1) % len(self.all_waypoints)
        next_heading_diff = self.get_waypoint_heading_diff(self.all_waypoints[next_i], position, orientation)
        if (optimized and min_heading_diff > math.pi/2 and next_heading_diff > math.pi/2):
            rospy.logerr('last_waypoint possibly incorrect, next iteration will search entire map')
            self.last_closest_wp_index = None
            return next_i, next_heading_diff
        elif(not optimized and min_heading_diff > math.pi/2):
            self.last_closest_wp_index = min_i+1
            return next_i, next_heading_diff
        else:
            self.last_closest_wp_index = min_i
            return min_i, min_heading_diff

    def get_car_heading(self, orientation):
        quaternion = (orientation.x,
                      orientation.y,
                      orientation.z,
                      orientation.w)
        rpy = tf.transformations.euler_from_quaternion(quaternion)
        return rpy[2]

    def pose_cb(self, msg):
        self.last_pose = msg.pose

    def waypoints_cb(self, data):
        self.all_waypoints = data.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def p2p_distance(self,a,b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def publish_waypoints(self):
        min_i = None
        position = self.last_pose
        min_i,min_h = self.get_closest_waypoint_index(position.position, position.orientation)

        rospy.loginfo("Curpos %f,%f,%f,%f h:%f, next waypoint is %d: %f,%f,%f,%f h:%f diff:%f", position.position.x,
                     position.position.y, position.orientation.z, position.orientation.w,
                     self.get_car_heading(position.orientation)
                     , min_i, self.all_waypoints[min_i].pose.pose.position.x,
                     self.all_waypoints[min_i].pose.pose.position.y,
                     self.all_waypoints[min_i].pose.pose.orientation.z,  self.all_waypoints[min_i].pose.pose.orientation.w,
                     self.get_car_heading(self.all_waypoints[min_i].pose.pose.orientation), min_h
                     )
        waypointCmds = []

        for i in range(min_i, min_i + LOOKAHEAD_WPS):
            waypoint = self.all_waypoints[i % len(self.all_waypoints)]

            # arbitrary slowdown on turns.
            waypoint.twist.twist.linear.x = min(12,4 + 4 / min_h)
            waypointCmds.append(waypoint)

        final_lane = Lane()
        currtime = rospy.Time.now()
        final_lane.header.stamp = currtime
        final_lane.waypoints = waypointCmds
        self.final_waypoints_pub.publish(final_lane)

    def loop(self):
        rate = rospy.Rate(30)  # reduced from 40 to 1 due to perf issues
        while not rospy.is_shutdown():
            if (self.last_pose and self.all_waypoints):
                self.publish_waypoints()
            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
