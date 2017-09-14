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


class WaypointUpdater(object):
    all_waypoints = None
    last_closest_wp_index = -1
    last_pose = None

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=10)

        # TODO: Add other member variables you need below
        self.loop()

    def get_closest_waypoint_index(self, position, orientation):
        heading = self.get_car_heading(orientation)
        min_distance = 1000
        min_i = 0
        for i in range(len(self.all_waypoints)):
            i = i % len(self.all_waypoints)
            waypoint = self.all_waypoints[i]
            distance = self.p2p_distance(position, waypoint.pose.pose.position)
            wp_heading = self.get_car_heading(waypoint.pose.pose.orientation)
            delta = abs(wp_heading - heading)

            if (distance < min_distance):
                min_distance = distance
                min_i = i
        return min_i

    def get_car_heading(self, orientation):
        quaternion = (orientation.x,
                      orientation.y,
                      orientation.z,
                      orientation.w)
        rpy = tf.transformations.euler_from_quaternion(quaternion)
        return rpy[2]

    def get_next_waypoint_index(self, position, orientation):
        closest = self.get_closest_waypoint_index(position, orientation)
        car_heading = self.get_car_heading(orientation)
        waypoint = self.all_waypoints[closest]
        theta = math.atan2(waypoint.pose.pose.position.y - position.y,
                      waypoint.pose.pose.position.x - position.x)
        heading_diff = abs(theta - car_heading)
        if (heading_diff > math.pi/4):
            closest += 1
            waypoint = self.all_waypoints[closest]
            newtheta = math.atan2(waypoint.pose.pose.position.y - position.y,
                      waypoint.pose.pose.position.x - position.x)
            new_diff = abs(newtheta - car_heading)
            rospy.logerr('Chose next one, Wp: %d, theta: %f, diff: %f, wp: %d, theta: %f, diff: %f', closest-1, theta, heading_diff, closest, newtheta, new_diff)
        else:
            rospy.logerr('Waypoint: %d, theta: %f, diff: %f', closest, theta, heading_diff)

        return closest

    def pose_cb(self, msg):
        self.last_pose = msg.pose
        #search for next closest waypoint, starting from index 0

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

    def loop(self):
        rate = rospy.Rate(30)  # reduced from 40 to 1 due to perf issues
        while not rospy.is_shutdown():
            if (self.last_pose):
                min_i = self.get_next_waypoint_index(self.last_pose.position, self.last_pose.orientation)

                self.last_closest_wp_index = min_i

                rospy.logerr("Curpos %f,%f,%f,%f h:%f, next waypoint is %d: %f,%f,%f,%f h:%f", self.last_pose.position.x,
                             self.last_pose.position.y, self.last_pose.orientation.z, self.last_pose.orientation.w,
                             self.get_car_heading(self.last_pose.orientation)
                             , min_i, self.all_waypoints[min_i].pose.pose.position.x,
                             self.all_waypoints[min_i].pose.pose.position.y,
                             self.all_waypoints[min_i].pose.pose.orientation.z,  self.all_waypoints[min_i].pose.pose.orientation.w,
                             self.get_car_heading(self.all_waypoints[min_i].pose.pose.orientation)
                             )
                waypointCmds = []

                for i in range(min_i, min_i + LOOKAHEAD_WPS):
                    waypoint = self.all_waypoints[i % len(self.all_waypoints)]
                    # waypoint.twist.twist.linear.x = 5

                    waypointCmds.append(waypoint)

                final_lane = Lane()
                currtime = rospy.Time.now()
                final_lane.header.stamp = currtime
                final_lane.waypoints = waypointCmds
                self.final_waypoints_pub.publish(final_lane)
            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
