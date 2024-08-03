#!/usr/bin/env python3    

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

current_pose = Pose()

def pose_callback(data):
    global current_pose
    current_pose = data

def move_to_goal(x_goal, y_goal):
    global current_pose

    rospy.init_node('turtle_move_to_goal', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rate = rospy.Rate(10)  # 1hz

    goal_pose = Pose()
    goal_pose.x = x_goal
    goal_pose.y = y_goal

    vel_msg = Twist()

    while not rospy.is_shutdown():
        distance = math.sqrt((goal_pose.x - current_pose.x)**2 + (goal_pose.y - current_pose.y)**2)
        vel_msg.linear.x = 1.5 * distance  
        angle_to_goal = math.atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x)
        angle_diff = angle_to_goal - current_pose.theta
        # Normalize the angle difference to [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
       
        vel_msg.angular.z = 4.0 * angle_diff
       

        if distance < 0.01:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            pub.publish(vel_msg)
            break

        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        x_goal = float(input("Enter the x goal for turtle1: "))
        y_goal = float(input("Enter the y goal for turtle1: "))
        move_to_goal(x_goal, y_goal)
    except rospy.ROSInterruptException:
        pass