#!/usr/bin/env python3
from math import pi
import rospy
import time
from geometry_msgs.msg import Twist

def control_velocity(control_linear_velx,control_angular_velz):
    twist = Twist()
    twist.linear.x = control_linear_velx; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_velz
    pub.publish(twist)
    return

if __name__=="__main__":
    rospy.init_node("adjust_location")
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    control_linear_vel = 1.0
    control_angular_vel = 0.0

    try:
        time_elapse = 0
        current_time = 0
        start_time = time.time()
        while not (rospy.is_shutdown() or (current_time>=5)):
            control_velocity(control_linear_vel, control_angular_vel)
            current_time = time.time() - start_time
    except:
        print("Not Found")
    finally:
        print("Tiempo transcurrido: %s" % (current_time))
        control_velocity(0,0)
