#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED = ONE_MPH
MAX_SPEED = 25 * ONE_MPH


'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    dbw_enabled = False
    brake_deadband = None
    decel_limit = None
    accel_limit = None
    wheel_radius = None
    vehicle_mass = None
    last_current_velocity = None
    last_current_velocity_ts = None
    last_throttle = None
    last_brake = None
    last_steering = None
    def __init__(self):
        rospy.init_node('dbw_node')

        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)


        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=10)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=10)

        # TODO: Create `TwistController` object
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.yaw_controller =  YawController(self.wheel_base, steer_ratio, 0, self.max_lat_accel, max_steer_angle)
        self.pid_enable_pub = rospy.Publisher('/throttle_pid/enable', Bool, queue_size=1)
        self.pid_state = rospy.Publisher('/throttle_pid/state', Float64, queue_size=1)
        self.pid_setpoint = rospy.Publisher('/throttle_pid/setpoint', Float64, queue_size=1)

        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.on_enabled)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.on_twist_cmd)
        rospy.Subscriber('/current_velocity', TwistStamped, self.on_current_velocity)

        rospy.Subscriber('/throttle_pid/control_effort', Float64, self.on_control_effort)
        rospy.spin()

    def on_control_effort(self, data):
        throttle = data.data
        tcmd = ThrottleCmd()
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT

        bcmd = BrakeCmd()
        #torque = Mass * acc * wheel_radius
        max_brake_torque = min(BrakeCmd.TORQUE_MAX, (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY)
                               * abs(self.decel_limit) * self.wheel_radius)

        if throttle >= 0:
            tcmd.enable = True
            bcmd.enable = False
            bcmd.boo_cmd = False
            tcmd.pedal_cmd = throttle
            bcmd.pedal_cmd = 0
        elif abs(throttle) > self.brake_deadband:
            # Must leave a deadband or the two parts of this PID controller will fight!
            bcmd.enable = True
            bcmd.boo_cmd = True
            tcmd.pedal_cmd = 0
            tcmd.enable = False
            #rospy.logerr('Max brake torque is %f', max_brake_torque)
            bcmd.pedal_cmd = (abs(throttle) - self.brake_deadband) * max_brake_torque
        else:
            # coasting
            tcmd.enable = False
            bcmd.enable = False

        if (self.last_throttle != tcmd.pedal_cmd):
            self.throttle_pub.publish(tcmd)
            self.last_throttle = tcmd.pedal_cmd

        if (self.last_brake != bcmd.pedal_cmd):
            self.brake_pub.publish(bcmd)
            self.last_brake = bcmd.pedal_cmd

    def on_current_velocity(self, data):
        self.last_current_velocity = data.twist.linear.x
        self.last_current_velocity_ts = rospy.Time.now()
        self.pid_state.publish(data.twist.linear.x)

    def on_enabled(self, data):
        self.dbw_enabled = data.data
        self.pid_enable_pub.publish(data.data)
        rospy.logerr('Got dbw_enabled : %s',str(data.data))

    def on_twist_cmd(self, data):
        # publish to PID control node and wait for output
        self.pid_setpoint.publish(data.twist.linear.x)

        if (self.last_current_velocity):
            steering = self.yaw_controller.get_steering(data.twist.linear.x, data.twist.angular.z, self.last_current_velocity)
            self.publish_steering(steering)

        #TODO: Handle angular velocity
        self.last_timestamp = rospy.Time(data.header.stamp.secs, data.header.stamp.nsecs)
        self.last_proposed_angular_vel = data.twist.angular.z
        self.last_proposed_vel = data.twist.linear.x
        #rospy.logerr('Got data: %s, last_ts: %f', str(data), self.last_timestamp.to_sec())

    def publish_steering(self, steer):
        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

if __name__ == '__main__':
    DBWNode()
