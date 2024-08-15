#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

def clear_costmaps():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clear_costmap_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_costmap_service()
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to clear costmaps: {e}")

def move_to_goal(x, y, w):
    # Create a SimpleActionClient for move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Define the goal position
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w

    # Clear costmaps before sending the goal
    clear_costmaps()

    # Send the goal to move_base
    rospy.loginfo(f"Sending goal to x: {x}, y: {y}, w: {w}")
    client.send_goal(goal)

    # Wait for the result
    wait = client.wait_for_result()
    
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal execution completed.")
        return client.get_result()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('move_turtlebot3_to_goal', anonymous=True)
        
        while not rospy.is_shutdown():
            # Get user input for goal coordinates
            x_goal = float(input("Enter the x coordinate of the goal: "))
            y_goal = float(input("Enter the y coordinate of the goal: "))
            w_orientation = float(input("Enter the orientation (w) of the goal: "))

            result = move_to_goal(x_goal, y_goal, w_orientation)

            if result:
                rospy.loginfo("Goal execution done!")
            else:
                rospy.logwarn("Failed to reach the goal. Please try again.")

            # Ask if the user wants to set another goal
            continue_input = input("Do you want to set another goal? (y/n): ").strip().lower()
            if continue_input != 'y':
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
    except ValueError:
        rospy.logerr("Invalid input! Please enter numeric values.")
