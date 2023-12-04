#!/usr/bin/env python3 
"""Osoyoo_controller controller."""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Camera
from controller import Lidar

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

R_motor = robot.getDevice("Right Motor")
L_motor = robot.getDevice("Left Motor")
steering = robot.getDevice("Steering")
cam_mot = robot.getDevice("Camera Servo")
cam = robot.getDevice("Pi_camera")
lidar = robot.getDevice("YDlidar")

cam.enable(50)
cam.getImage()

lidar.enable(1)
lidar.setFrequency(8)
lidar.enablePointCloud()


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

linear_vel = 0.33
angular_vel = 0.0
wheel_radius = 0.034


def callback(data):
    global linear_vel 
    global angular_vel 
    rospy.loginfo(rospy.get_caller_id() + " - linear: %f, angular: %f", data.linear.x , data.angular.z)
    linear_vel = data.linear.x
    angular_vel = data.angular.z 

def cmd_listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('cmd_listener', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    #spin() simply keeps python from exiting until this node is stopped
   # rospy.spin()

cmd_listener() 

# Main loop:
# - perform simulation steps until Webots is stopping the controller

L_motor.setPosition(float('Inf'))
L_motor.setVelocity(0.0)
R_motor.setPosition(float('Inf'))
R_motor.setVelocity(0.0)

while robot.step(timestep) != -1:
    wheel_vel = linear_vel/wheel_radius
    
    steering.setPosition(-0.01)
    L_motor.setVelocity(whee1_vel)
    R_motor.setVelocity(wheel_vel)
    print(wheel_vel)

   # lidar_data = lidar.getRangeImage()
   # print (lidar_data)
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    # val = ds.getValue()
    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
cam.disable