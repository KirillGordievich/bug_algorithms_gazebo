#!/usr/bin/python
import rospy
import math

from geometry_msgs.msg import Twist

def bug_0(self):
    if self.can_head_toward_goal() is True: # go to point
        self.change_state(0)
        self.go_to_point(self.get_goal())   
    else: # follow the wall
        self.follow_the_wall()
