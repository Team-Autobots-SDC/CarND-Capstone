
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED = ONE_MPH
MAX_SPEED = 25 * ONE_MPH
KP = 0.7
KI = 0.002
KD = 5.2816
from yaw_controller import YawController
from pid import PID
import rospy

class Controller(object):

    def __init__(self, vehicle_mass, brake_deadband, decel_limit,accel_limit, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_SPEED, max_lat_accel, max_steer_angle)
        self.vel_pid_controller = PID(KP,KI,KD, -1, 1)
        self.steering_pid_controller = PID(KP,KI,KD, -max_steer_angle, max_steer_angle)
        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit

    def control(self, target_velocity, target_angular_velocity, current_velocity, time_elapsed_s):
        steering = self.yaw_controller.get_steering(target_velocity, target_angular_velocity, current_velocity)
        rospy.logerr('time_elapsed: %f', time_elapsed_s)
        throttle = self.vel_pid_controller.step(target_velocity - current_velocity, time_elapsed_s)
        if (throttle < 0):
            braking = - throttle
            throttle = 0
        else:
            braking = 0

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return throttle,braking, steering
