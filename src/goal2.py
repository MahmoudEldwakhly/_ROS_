#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Initialize global variables
current_pose1 = Pose()
current_pose2 = Pose()

def pose_callback1(data):
    global current_pose1
    current_pose1 = data

def pose_callback2(data):
    global current_pose2
    current_pose2 = data

def move_to_goal(x_goal1, y_goal1, x_goal2, y_goal2):
    global current_pose1
    global current_pose2

    rospy.init_node('turtle_move_to_goal', anonymous=True)
    pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    sub1 = rospy.Subscriber('/turtle1/pose', Pose, pose_callback1)

    pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
    sub2 = rospy.Subscriber('/turtle2/pose', Pose, pose_callback2)

    rate = rospy.Rate(10)  # 10hz

    goal_pose1 = Pose()
    goal_pose2 = Pose()
    goal_pose1.x = x_goal1
    goal_pose1.y = y_goal1

    goal_pose2.x = x_goal2
    goal_pose2.y = y_goal2

    vel_msg1 = Twist()
    vel_msg2 = Twist()

    while not rospy.is_shutdown():
        # Calculate distance and velocity for turtle1
        distance1 = math.sqrt((goal_pose1.x - current_pose1.x)**2 + (goal_pose1.y - current_pose1.y)**2)
        vel_msg1.linear.x = 1.5 * distance1
        angle_to_goal1 = math.atan2(goal_pose1.y - current_pose1.y, goal_pose1.x - current_pose1.x)
        angle_diff1 = angle_to_goal1 - current_pose1.theta
        angle_diff1 = (angle_diff1 + math.pi) % (2 * math.pi) - math.pi
        vel_msg1.angular.z = 4.0 * angle_diff1

        # Calculate distance and velocity for turtle2
        distance2 = math.sqrt((goal_pose2.x - current_pose2.x)**2 + (goal_pose2.y - current_pose2.y)**2)
        vel_msg2.linear.x = 1.5 * distance2
        angle_to_goal2 = math.atan2(goal_pose2.y - current_pose2.y, goal_pose2.x - current_pose2.x)
        angle_diff2 = angle_to_goal2 - current_pose2.theta
        angle_diff2 = (angle_diff2 + math.pi) % (2 * math.pi) - math.pi
        vel_msg2.angular.z = 4.0 * angle_diff2

        # Stop turtle1 when it reaches the goal
        if distance1 < 0.01:
            vel_msg1.linear.x = 0
            vel_msg1.angular.z = 0
            pub1.publish(vel_msg1)
        else:
            pub1.publish(vel_msg1)

        # Stop turtle2 when it reaches the goal
        if distance2 < 0.01:
            vel_msg2.linear.x = 0
            vel_msg2.angular.z = 0
            pub2.publish(vel_msg2)
        else:
            pub2.publish(vel_msg2)

        rate.sleep()

if __name__ == '__main__':
    try:
        x_goal1 = float(input("Enter the x goal for turtle1: "))
        y_goal1 = float(input("Enter the y goal for turtle1: "))
        x_goal2 = float(input("Enter the x goal for turtle2: "))
        y_goal2 = float(input("Enter the y goal for turtle2: "))

        move_to_goal(x_goal1, y_goal1, x_goal2, y_goal2)
    except rospy.ROSInterruptException:
        pass
