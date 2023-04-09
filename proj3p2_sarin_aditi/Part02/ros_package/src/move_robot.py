#!/usr/bin/env python3
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math

# Definining the X and Y coordinates to be travelled by the robot based on the generated shortest path

# pose_list = [(0.0, 0.0, 0.0, 7.329, 14.658), (0.5, 0.0, 0, 14.658, 14.658), 
#              (0.8, -0.2, -100, 14.658, 7.329), (1.0, -0.5, 360, 7.329, 14.658), 
#              (1.5, -0.5, 0, 14.658, 14.658), (1.8, -0.3, 100, 7.329, 14.658), 
#              (1.8, -0.0, 100, 7.329, 7.329), (1.8, 0.3, 100, 7.329, 7.329), 
#              (2.0, 0.6, 0, 14.658, 7.329), (2.5, 0.6, 0, 14.658, 14.658), 
#              (3.0, 0.6, 0, 14.658, 14.658), (3.1, 0.7, 100, 0, 7.329), 
#              (3.2, 0.8, 0, 7.329, 0), (3.7, 0.8, 0, 14.658, 14.658), 
#              (4.0, 0.6, -100, 14.658, 7.329), (4.1, 0.5, 360, 0, 7.329), 
#              (4.4, 0.3, -100, 14.658, 7.329), (4.6, 0.0, 360, 7.329, 14.658), 
#              (4.9, 0.0, 0, 7.329, 7.329)]

# def read_path_list(input_array):
#     coordinates = []
#     for item in input_array:
#         coordinates.append([item[0], item[1]])    
#     return coordinates


#Reading file from file path. Make sure to change the file path based on where you download the file. 
def read_file_path(filename='/home/sarin/Documents/661/Project3_phase1/proj3_p2_sarin_aditi/generated_path.txt'):
    coordinates = []
    with open(filename, 'r') as file_name:
        lines = file_name.readlines()
    for line in lines:
        x, y, th, left_rpm, right_rpm = line.strip().split('\t') #This line takes the points from the path separated by spaces
        x = round(float(x), 2)
        y = round(float(y), 2)
        # coordinates.append([float(x), float(y)])
        coordinates.append([x, y])
    return coordinates

#Defining the initial coordinates of the robot as defined in the launch file
x_coord = 0.0
y_coord = 0.0
theta = 0.0

def orientation_between_nodes(vel_msg):
    global x_coord, y_coord, theta
    x_coord = vel_msg.pose.pose.position.x
    y_coord = vel_msg.pose.pose.position.y
    quaternion = (
        vel_msg.pose.pose.orientation.x,
        vel_msg.pose.pose.orientation.y,
        vel_msg.pose.pose.orientation.z,
        vel_msg.pose.pose.orientation.w
    )
    _, _, theta = euler_from_quaternion(quaternion)


def turtlebot_motion(goal_x_coord, goal_y_coord):
    publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    vel_msg = Twist()
    goal_reached = False

    while not goal_reached and not rospy.is_shutdown():

        delta_y = goal_y_coord - y_coord
        delta_x = goal_x_coord - x_coord
        
        rotation = math.atan2(delta_y, delta_x)
        dist = math.sqrt(delta_x**2 + delta_y**2)

        if abs(rotation - theta) > 0.3:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = rotation - theta
        
        else:
            vel_msg.linear.x = min(0.5, dist)
            vel_msg.angular.z = 0.0

            if dist < 0.01:
                goal_reached = True
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0

        publisher.publish(vel_msg)
        rate.sleep()

if __name__ == "__main__":
    # rospy.init_node(read_path'turtlebot_move')
    rospy.init_node('move_turtlebot')
    odom_sub = rospy.Subscriber('/odom', Odometry, orientation_between_nodes)
    # poses = read_path_list(pose_list)
    poses = read_file_path()
    # poses = read_path()
    for pose in poses:
        temp_goal_x, temp_goal_y = pose
        turtlebot_motion(temp_goal_x, temp_goal_y)
