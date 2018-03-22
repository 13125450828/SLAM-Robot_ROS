#!/usr/bin/env python

import rospy
import actionlib
import tf
import os

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy

'''waypoints = [
    [(0.214, 2.247, 0.0), (0.0, 0.0, -0.316, 0.948)],
    [(3.054, 1.948, 0.0), (0.0, 0.0, 0.384, 0.923)], 
    [(3.483, 6.468, 0.0), (0.0, 0.0, 0.898, 0.441)], 
    [(1.149, 7.04, 0.0), (0.0, 0.0, -0.982, 0.191)],
    [(0.214, 2.247, 0.0), (0.0, 0.0, -0.316, 0.948)]
]'''

waypoints = [
		[(-0.547, 1.834, 0.0), (0.0, 0.0, -0.235, 0.972)],
    [(2.726, 1.313, 0.0), (0.0, 0.0, 0.391, 0.920)], 
    [(3.619, 7.674, 0.0), (0.0, 0.0, 0.903, 0.429)], 
    [(0.655, 8.02, 0.0), (0.0, 0.0, -0.969, 0.244)],
    [(-0.847, 1.834, 0.0), (0.0, 0.0, -0.235, 0.972)]
    #[(-0.643, 2.273, 0.0), (0.0, 0.0, -0.679, 0.734)]
]


client = None
curr_pose = None
running = False
loop_check = False

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose


def joy_callback(msg):
    global running
    global loop_check
    global client
    if msg.buttons[0]==1:
        running = not running
        print "MOVE"
    elif msg.buttons[1]==1:
        print 'Exit...'
        
        client.cancel_goal()
        loop_check = not loop_check
        print "cancel"
        #os._exit(0)


if __name__ == '__main__':
    rospy.init_node('patrol')
    joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
    client.wait_for_server()

    # Go to the start point:
    #client.send_goal(goal_pose(waypoints[0]))
    #client.wait_for_result()
    
    while not running:
        print "WHILE"
        pass
        

    print 'ready'
    i=1   	
    for num in range(1,3):
    	
		  for pose in waypoints:
		      curr_pose = pose
		      goal = goal_pose(pose)
		      print 'going . . .'
		      print i
		      i = i + 1
		      client.send_goal(goal)
		      print "REACHED GOAL"
		      client.wait_for_result()
		      if loop_check:
		        break



