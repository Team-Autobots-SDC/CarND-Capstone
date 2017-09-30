#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import TrafficLight
from styx_msgs.msg import Lane, Waypoint
from jmt import JMT
import tf.transformations
import math
import copy
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_SEARCH_WPS = 100

class WaypointUpdater(object):
    all_waypoints = None
    waypoints_s = None
    last_closest_wp_index = None
    last_pose = None

    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.loop_rate = rospy.get_param('~loop_rate', 5.)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        self.basepoint_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.traffic_light_sub = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_light_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=10)
        self.light_wp = -1
        self.jmt = None
        self.last_vel = None

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

        # So the logic goes like this, if using a optimized search waypoint AND you get both closest heading
        # and next heading diff to be very large, something MIGHT have gone wrong and you have lost the
        # closest waypoint, so reset and start from scratch on the next iteration.
        # However, if not optimized and your current min waypoint is offset by more than PI/2, then the next waypoint
        # is the best option so just pick it (regardless if its optimized or not).
        # If < PI/2 then this is the correct waypoint so go with it.

        if (optimized and min_heading_diff > math.pi/2.0 and next_heading_diff > math.pi/2.0):
            rospy.logerr('last_waypoint possibly incorrect, next iteration will search entire map')
            self.last_closest_wp_index = None
            return next_i, next_heading_diff
        elif(min_heading_diff > math.pi/2):
            rospy.loginfo('Returning next one, wp :%d, diff: %f, nextwp: %d, diff: %f', min_i, min_heading_diff, next_i, next_heading_diff)
            self.last_closest_wp_index = next_i
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

    def velocity_cb(self, msg):
        self.last_vel = msg.twist

    def waypoints_cb(self, data):
        self.all_waypoints = data.waypoints
        s = 0
        self.waypoints_s = []
        self.waypoints_s.append(0)
        for i in range(0, len(self.all_waypoints)-1):
            s += self.distance(self.all_waypoints, i, i+1)
            self.waypoints_s.append(s)

        self.basepoint_sub.unregister()

    def traffic_light_cb(self, data):
        self.light_wp = data.data

    # The following two functions have been monkey-ported over from the Path Planning project


    def calculate_frenet(self, position, orientation):
        next_wp_idx, heading = self.get_closest_waypoint_index(position, orientation)
        prev_wp_idx = next_wp_idx - 1
        if (next_wp_idx == 0):
            prev_wp_idx = len(self.all_waypoints) - 1

        next_wp_pos = self.all_waypoints[next_wp_idx].pose.pose.position
        prev_wp_pos = self.all_waypoints[prev_wp_idx].pose.pose.position

        n_x = next_wp_pos.x - prev_wp_pos.x
        n_y = next_wp_pos.y - prev_wp_pos.y
        x_x = position.x - prev_wp_pos.x
        y_y = position.y - prev_wp_pos.y

        #Find projection
        proj_norm = (x_x * n_x + y_y * n_y) / (n_x ** 2 + n_y ** 2)
        proj_x = proj_norm * n_x
        proj_y = proj_norm * n_y

        frenet_d = self.p2p_distance_xy(x_x, y_y, proj_x, proj_y)

        center_x = 1000 - prev_wp_pos.x
        center_y = 2000 - prev_wp_pos.y

        centerToPos = self.p2p_distance_xy(center_x, center_y, x_x, y_y)
        centerToRef = self.p2p_distance_xy(center_x, center_y, proj_x, proj_y)

        if (centerToPos <= centerToRef):
            frenet_d *= -1

        #calculate s
        frenet_s = self.distance(self.all_waypoints, 0 , prev_wp_idx) + self.p2p_distance_xy(0,0, proj_x, proj_y)
        return (frenet_s, frenet_d)

    def getXY(self, s, d):
        prev_wp = -1
        while (prev_wp < (len(self.waypoints_s)-1)) and (s > self.waypoints_s[prev_wp+1]):
            prev_wp += 1
        next_wp = (prev_wp+1) % len(self.waypoints_s)
        next_wp_pos = self.all_waypoints[next_wp].pose.pose.position
        prev_wp_pos = self.all_waypoints[prev_wp].pose.pose.position
        heading = math.atan2(next_wp_pos.y - prev_wp_pos.y, next_wp_pos.x - prev_wp_pos.x)
        seg_s = s - self.waypoints_s[prev_wp]

        seg_x = prev_wp_pos.x + seg_s * math.cos(heading)
        seg_y = prev_wp_pos.y + seg_s * math.sin(heading)

        normal_heading = heading - math.pi / 2
        x = seg_x + d * math.cos(normal_heading)
        y = seg_y + d * math.sin(normal_heading)

        return (x,y)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def p2p_distance_xy(self,ax,ay, bx,by):
        return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)

    def p2p_distance(self,a,b):
        return self.p2p_distance_xy(a.x, a.y, b.x, b.y)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1_index, wp2_index):
        """
            Piece wise distance calculation. Starts at the wp1_index and calculates
            the total distance between wp1_index and wp2_index.
        """
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1_index, wp2_index+1):
            dist += dl(waypoints[wp1_index].pose.pose.position, waypoints[i].pose.pose.position)
            wp1_index = i
        return dist

    def publish_waypoints(self):
        min_i = None
        position = self.last_pose
        min_i,min_h = self.get_closest_waypoint_index(position.position, position.orientation)

        s, d = self.calculate_frenet(position.position, position.orientation)
        x, y = self.getXY(s, d)
        rospy.loginfo(
            "Curpos %f,%f,%f,%f h:%f,s:%f, d:%f, cx:%f, cy: %f,  next waypoint is %d: %f,%f,%f,%f h:%f diff:%f",
            position.position.x,
            position.position.y, position.orientation.z, position.orientation.w,
            self.get_car_heading(position.orientation),
            s, d, x, y,
            min_i, self.all_waypoints[min_i].pose.pose.position.x,
            self.all_waypoints[min_i].pose.pose.position.y,
            self.all_waypoints[min_i].pose.pose.orientation.z,
            self.all_waypoints[min_i].pose.pose.orientation.w,
            self.get_car_heading(self.all_waypoints[min_i].pose.pose.orientation), min_h
        )

        waypointCmds = []

        #rospy.logerr('Updating waypoints in range [%d-%d]', min_i, min_i + LOOKAHEAD_WPS)

        waypoint_r = range(min_i, min_i + LOOKAHEAD_WPS)
        stop_at_light = self.light_wp in waypoint_r

        if stop_at_light:
            first = self.jmt is None

            # calculate distance to light
            s_start = self.waypoints_s[waypoint_r[0] % len(self.all_waypoints)]
            s_end = self.waypoints_s[self.light_wp % len(self.all_waypoints)]
            dist = abs(s_end - s_start)

            # calculate estimated time interval of arrival
            speed = self.last_vel.linear.x if self.last_vel is not None else 0

            if self.jmt == None:
                rospy.loginfo('Stopping at light: %d', self.light_wp)

                accel = 0 # XXX assume 0 for now
                ttl = (dist / speed)

                # generate new jmt
                rospy.loginfo('   estimated speed: %f, ttl %f, s_start %f, s_end %f', speed, ttl, s_start, s_end)

                """
                jmts = JMT.search_jmts([s_start, speed, accel ], # start state
                                       [s_end, 0, 0], # end state mean
                                       [dist, 2, 2], # end state std dev
                                       [speed, 10, 10], # max speed, accel, jerk
                                       100, # number of samples
                                       ttl,
                                       0.1, # sampling rate
                                       0.0 # time period deviation
                                        )
                                        
                # take best one in terms of score
                self.jmt = jmts[0]
                                        
                                        """

                self.jmt = JMT([s_start, speed, accel], [s_end, 0, 0], ttl)

                # JMT is not fully working... linearize for now
                self.jmt.linearize(-speed/ttl)


            for i in waypoint_r:
                waypoint = self.all_waypoints[i % len(self.all_waypoints)]
                waypoint_s = self.waypoints_s[i % len(self.all_waypoints)]
                # adjust waypoints from current waypoint to light_wp (and beyond)
                waypoint = copy.deepcopy(waypoint)  # deep copy waypoint to avoid changing velocity on source data
                if i < self.light_wp:
                    t = self.jmt.time_for_position(waypoint_s)
                    waypoint.twist.twist.linear.x = self.jmt.speed(t)
                    if first:
                        rospy.loginfo('   wp: %d, speed %f, t %f, s %f', i, waypoint.twist.twist.linear.x, t, waypoint_s)
                else:
                    waypoint.twist.twist.linear.x = 0
                waypointCmds.append(waypoint)

        else:
            if self.jmt is not None:
                rospy.loginfo('Light is no longer red, moving forward')
                self.jmt = None
            for i in waypoint_r:
                waypoint = self.all_waypoints[i % len(self.all_waypoints)]
                waypointCmds.append(waypoint)

        final_lane = Lane()
        currtime = rospy.Time.now()
        final_lane.header.stamp = currtime
        final_lane.waypoints = waypointCmds
        self.final_waypoints_pub.publish(final_lane)

    def loop(self):
        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            if (self.last_pose and self.all_waypoints):
                self.publish_waypoints()
            rate.sleep()

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
