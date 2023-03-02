#!/usr/bin/env python3

import rospy, math
import numpy
from math import pi
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64

class Ackermann_Controller(object):
    def __init__(self):
        rospy.init_node('cmd_vel_to_motors')
        self._sleep_timer = rospy.Rate(self._DEF_PUB_FREQ)
        self._last_cmd_time = rospy.get_time()
        self._cmd_timeout = self._DEF_CMD_TIMEOUT
        self._wheelbase = 0.2
        self._steer_ang = 0.0      # Steering angle
        self._ang = 0.0
        self._steer_ang_vel = 0.0  # Steering angle velocity
        self._last_steer_ang = 0.0  # Last steering angle
        self._speed = 0.0
        self._last_speed = 0.0
        #self._accel = 0.0          # Acceleration
        self._joint_dist_div_2 = 0.095
        self._left_rear_ang_vel = 0.0
        self._right_rear_ang_vel = 0.0
        self._matrix = Float32MultiArray()
        #self._pub_left = rospy.Publisher('left_rear', Float64, queue_size=1)
        #self._pub_right = rospy.Publisher('right_rear', Float64, queue_size=1)
        #self._pub_angle = rospy.Publisher('angle', Float64, queue_size=1)
        self._pub = rospy.Publisher('Data_Ctrl_Motors', Float32MultiArray, queue_size=1)
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_callback)
        #rospy.Subscriber(name, data_class)
    def spin(self):
        last_time = rospy.get_time()
        while not rospy.is_shutdown():
            t = rospy.get_time()
            delta_t = t - last_time
            last_time = t

            if (self._cmd_timeout > 0.0 and t - self._last_cmd_time > self._cmd_timeout):
                # Too much time has elapsed since the last command. Stop the
                # vehicle.
                steer_ang_changed, radius_robot = self._ctrl_steering(self._last_steer_ang, 0.0, 0.001)
                self._ctrl_axles(0.0, steer_ang_changed, radius_robot)
            elif delta_t > 0.0:
                steer_ang = self._steer_ang
                steer_ang_vel = self._steer_ang_vel
                speed = self._speed
                #accel = self._accel
                steer_ang_changed, radius_robot = self._ctrl_steering(steer_ang, steer_ang_vel, delta_t)
                self._ctrl_axles(speed, steer_ang_changed, radius_robot)
            self._matrix.data = [self._ang, self._left_rear_ang_vel, self._right_rear_ang_vel]
            #self._pub_left.publish(self._left_rear_ang_vel)
            #self._pub_right.publish(self._right_rear_ang_vel)
            #self._pub_angle.publish(self._ang)
            self._pub.publish(self._matrix)
            self._sleep_timer.sleep()
    _DEF_WHEEL_DIA = 0.065    # Default wheel diameter. Unit: meter.
    _DEF_EQ_POS = 0.0       # Default equilibrium position. Unit: meter.
    _DEF_CMD_TIMEOUT = 0.5  # Default command timeout. Unit: second.
    _DEF_PUB_FREQ = 30.0    # Default publishing frequency. Unit: hertz.


    def _convert_trans_rot_vel_to_steering_angle(self,v, omega, wheelbase):
        if omega == 0 or v == 0:
            return 0
        radius = v / omega
        return math.atan(wheelbase / radius)

    def cmd_callback(self, data):
        #global wheelbase
        self._last_cmd_time = rospy.get_time()
        self._speed = data.linear.x
        self._steer_ang_vel = data.angular.z
        self._steer_ang = self._convert_trans_rot_vel_to_steering_angle(self._speed, self._steer_ang_vel, self._wheelbase)
    
    def _ctrl_steering(self,steer_ang,steer_ang_vel_limit,delta_t):
        if steer_ang_vel_limit > 0.0:
            # Limit the steering velocity.
            ang_vel = (steer_ang - self._last_steer_ang) / delta_t
            ang_vel = max(-steer_ang_vel_limit, min(ang_vel, steer_ang_vel_limit))
            theta = self._last_steer_ang + ang_vel * delta_t
        else:
            theta = steer_ang
        radius_robot = self._wheelbase*math.tan((pi/2)-theta)*0.5
        steer_ang_changed = theta != self._last_steer_ang
        if steer_ang_changed:
            self._last_steer_ang = theta
            self._ang = self._get_steer_ang(math.atan(radius_robot/self._wheelbase))
        return steer_ang_changed, radius_robot

    def _ctrl_axles(self, speed, steer_ang_changed, radius_robot):
        veh_speed = speed
        if veh_speed != self._last_speed or steer_ang_changed:
            self._last_speed = veh_speed
            left_dist = radius_robot - self._joint_dist_div_2
            right_dist = radius_robot + self._joint_dist_div_2
            # Rear
            gain = (2 * pi) * veh_speed / radius_robot
            self._left_rear_ang_vel = gain * left_dist /(pi*self._DEF_WHEEL_DIA)
            self._right_rear_ang_vel = gain * right_dist /(pi*self._DEF_WHEEL_DIA)
    def _get_steer_ang(self, phi):
        if phi >= 0.0:
            return (pi / 2) - phi
        return (-pi / 2) - phi
if __name__=='__main__':
    ctrlr = Ackermann_Controller()
    ctrlr.spin()
