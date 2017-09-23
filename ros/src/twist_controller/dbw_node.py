#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64, Float32
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from yaw_controller import YawController
from pid import PID

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

class State(object):

    def __init__(self, val):
        self.value = val
        self.ts = rospy.Time.now()

    def update(self, val):
        retval = not(self.equals(val))
        if retval:
            #rospy.logerr('values not equal: {}, {}'.format(self.value, val))
            self.value = val
            self.ts = rospy.Time.now()
        return retval

    def reset(self):
        # set of a non-equivalent value
        self.value -= 0.0011

    def equals(self, val):
        return abs(self.value - val) <= 0.001

class DBWNode(object):
    dbw_enabled = False
    brake_deadband = None
    decel_limit = None
    accel_limit = None
    wheel_radius = None
    vehicle_mass = None

    def __init__(self):
        rospy.init_node('dbw_node')

        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = rospy.get_param('~brake_deadband', .02)
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        self.loop_rate = rospy.get_param('~loop_rate', 5.)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=10)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=10)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=10)

        # TODO: Create `TwistController` object

        self.yaw_controller =  YawController(self.wheel_base, steer_ratio, 0, self.max_lat_accel, max_steer_angle)

        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.on_enabled)
        rospy.Subscriber('/vehicle/throttle_report', Float32, self.on_throttle_report)
        rospy.Subscriber('/vehicle/brake_report', Float32, self.on_brake_report)
        rospy.Subscriber('/vehicle/steering_report', SteeringReport, self.on_steering_report)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.on_twist_cmd)
        rospy.Subscriber('/current_velocity', TwistStamped, self.on_current_velocity)

        rospy.Subscriber('/throttle_pid/control_effort', Float64, self.on_control_effort)
        self.pid_enable_pub = rospy.Publisher('/throttle_pid/enable', Bool, queue_size=1)
        self.pid_state = rospy.Publisher('/throttle_pid/state', Float64, queue_size=1)
        self.pid_setpoint = rospy.Publisher('/throttle_pid/setpoint', Float64, queue_size=1)
        self.speed = State(0)
        self.brake = State(0)
        self.throttle = State(0)
        self.proposed_speed = 0.0

        self.loop()

    def on_control_effort(self, data):
        throttle = data.data
        brake_torque = 0.0

        #torque = Mass * acc * wheel_radius
        max_brake_torque = min(BrakeCmd.TORQUE_MAX, (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY)
                               * abs(self.decel_limit) * self.wheel_radius)

        #rospy.logerr('throttle %f', throttle)

        if throttle > 0:
            pass

        elif abs(throttle) > self.brake_deadband:
            # Must leave a deadband or the two parts of this PID controller will fight!
            brake_torque = max(0, abs(throttle) - self.brake_deadband) * max_brake_torque
            throttle = 0.0

        else:
            # coasting
            throttle = 0.0
            brake_torque = 0.0

        self.publish_throttle(throttle)
        self.publish_brake(brake_torque)

    def publish_throttle(self, throttle_perc):

        assert (0.0 <= throttle_perc <= 1.0)

        if self.throttle.update(throttle_perc):
            cmd = ThrottleCmd()
            cmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            cmd.pedal_cmd = throttle_perc
            cmd.enable = True

            #rospy.logerr('throttle pedal_cmd is %f', throttle_perc)

            self.throttle_pub.publish(cmd)


    def publish_brake(self, brake_torque):

        assert (0.0 <= brake_torque <= BrakeCmd.TORQUE_MAX)

        if self.brake.update(brake_torque):
            cmd = BrakeCmd()
            cmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
            cmd.pedal_cmd = brake_torque
            cmd.boo_cmd = True
            cmd.enable = True

            #rospy.logerr('brake pedal_cmd is %f, enable %d', brake_torque, cmd.enable)

            self.brake_pub.publish(cmd)


    def on_current_velocity(self, data):
        speed = data.twist.linear.x
        self.speed.update(speed)

    def on_enabled(self, data):
        self.dbw_enabled = data.data
        self.pid_enable_pub.publish(data.data)
        rospy.logerr('Got dbw_enabled : %s',str(data.data))

    def on_brake_report(self, msg):
        if not(self.brake.equals(msg.data)):
            # reset and republish
            brake = self.brake.value
            self.brake.reset()
            self.publish_brake(brake)

    def on_throttle_report(self, msg):
        if not(self.throttle.equals(msg.data)):
            # reset and republish
            throttle = self.throttle.value
            self.throttle.reset()
            self.publish_throttle(throttle)

    def on_steering_report(self, msg):
        pass

    def on_twist_cmd(self, data):
        rospy.logerr('Got twist cmd : %s', str(data))

        # do not allow the car to go backwards
        speed = max(0, data.twist.linear.x)

        rospy.logerr('New speed : %s', speed)

        # publish to PID control node and wait for output
        self.pid_setpoint.publish(speed)

        #TODO: Handle angular velocity
        #self.last_timestamp = rospy.Time(data.header.stamp.secs, data.header.stamp.nsecs)
        #self.last_proposed_angular_vel = data.twist.angular.z

        self.proposed_speed = speed
        #rospy.logerr('Got data: %s, last_ts: %f', str(data), self.last_timestamp.to_sec())

    def loop(self):
        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            pid_enable = self.dbw_enabled and not (self.proposed_speed == 0.0 and self.speed.value < 0.01)
            self.pid_enable_pub.publish(pid_enable)
            self.pid_state.publish(self.speed.value)
            rate.sleep()

    def publish_steering(self, steer):
        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

if __name__ == '__main__':
    DBWNode()
