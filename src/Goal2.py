#!/usr/bin/env python3    

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
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

def move_to_goal(pub, current_pose, x_goal, y_goal):
    rate = rospy.Rate(10)  # 10hz

    goal_pose = Pose()
    goal_pose.x = x_goal
    goal_pose.y = y_goal

    vel_msg = Twist()

    while not rospy.is_shutdown():
        # Calculate the distance to the goal
        distance = math.sqrt((goal_pose.x - current_pose.x)**2 + (goal_pose.y - current_pose.y)**2)

        # Calculate the linear velocity
        vel_msg.linear.x = 1.5 * distance

        # Calculate the angle to the goal
        angle_to_goal = math.atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x)

        angle_diff = angle_to_goal - current_pose.theta
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # Calculate the angular velocity
        vel_msg.angular.z = 4.0 * angle_diff

        # Stop the turtle when it reaches the goal
        if distance < 0.01:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            pub.publish(vel_msg)
            return

        # Publish the velocity message
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('multi_turtle_move', anonymous=True)

        x_goal1 = float(input("Enter the x goal for turtle1: "))
        y_goal1 = float(input("Enter the y goal for turtle1: "))
        x_goal2 = float(input("Enter the x goal for turtle2: "))
        y_goal2 = float(input("Enter the y goal for turtle2: "))

        pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        sub1 = rospy.Subscriber('/turtle1/pose', Pose, pose_callback1)

        rospy.wait_for_service('/spawn')
        spawner = rospy.ServiceProxy('/spawn', Spawn)
        spawner(5, 5, 0, 'turtle2')

        pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        sub2 = rospy.Subscriber('/turtle2/pose', Pose, pose_callback2)

        rospy.sleep(1)  # Allow some time for subscribers to get the first pose messages

        # Move both turtles to their respective goals
        move_to_goal(pub1, current_pose1, x_goal1, y_goal1)
        move_to_goal(pub2, current_pose2, x_goal2, y_goal2)
        
        rospy.spin()
            
    except rospy.ROSInterruptException:
        pass
