#!/usr/bin/env python3
import rospy
import signal
import OsoyooPi_controller as car
from geometry_msgs.msg import Twist
import time

previous_linear_vel = 0.0
linear_vel = 0.0
angular_vel = 0.0

print_info = False

def cmd_callback(data):
	global previous_linear_vel
	global linear_vel
	global angular_vel
	global print_info
	previous_linear_vel = linear_vel
	linear_vel = data.linear.x
	angular_vel = data.angular.z
	if print_info == True:
		rospy.loginfo("lineare %f, angolare %f, prev_lineare %f", linear_vel, angular_vel, previous_linear_vel)
	car.drive(linear_vel, angular_vel, previous_linear_vel, TEST_print_pwm = print_info)

def cmd_listener():
	rospy.init_node('cmd_vel_listener', anonymous=True, disable_signals=True)
	rospy.Subscriber("cmd_vel", Twist, cmd_callback)
	rospy.spin()

car.center_all()
cmd_listener()
rospy.on_shutdown(car.end_switch_off)
