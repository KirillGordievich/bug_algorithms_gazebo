#!/usr/bin/python
import rospy
import math

from geometry_msgs.msg import Twist

def bug_2(self):
    d = self.safe_distance
    min_d = self.min_safe_distance
    pose = self.get_pose()
    x_goal, y_goal = self.get_goal()
    epsilon = 0.3
    
    if self.state == 0: # go to point
        if self.regions['front'] < d - epsilon: # if meet an obstacle
            self.count_time = 0 # reset timer
            self.count_loop = 0
            self.change_state(1) # follow the obstacle/wall
                
            self.hit_point = pose # remember the point where we hit obstacle
            rospy.loginfo('self.hit_point = %s' % self.hit_point) 
        else:
            self.go_to_point([x_goal, y_goal]) # go to point
                
    elif self.state in [1,2,3]: # 1, 2 and 3 are "follow_the_wall" states
        # dist2 is the distance between old hit point and the goal
        dist2 = self.get_distance(self.hit_point[0], self.hit_point[1], x_goal, y_goal)
        # dist2 is the distance between the current position point and the goal
        dist1 = self.get_distance(pose[0], pose[1], x_goal, y_goal)
        message =  'self.get_distance_to_line(pose) = %s, abs(dist1 - dist2) = %s'
        rospy.loginfo_throttle(2, message % (self.get_distance_to_line(pose), dist1 - dist2))
        #if met the start-goal line and more than 3 seconds passed
        # and the distance from the new hit point to the goal is less than the distance from old hit point
        if (self.get_distance_to_line(pose) < 0.15 and 
            self.count_time > 2 and
            self.can_head_toward_goal() and 
            dist1 < dist2 - 0.15 and 
            dist1 < dist2 + 0.15): 
            # while robot heading differs from the desired heading by more than the yaw precision
            while math.fabs(self.get_err_yaw()) > self.yaw_precision:
                linear, angular = 0, -0.25
                self.vel_msg.linear.x = linear
                self.vel_msg.angular.z = angular
                self.velocity_pub.publish(self.vel_msg)
                rospy.loginfo_throttle(1,'Yaw error: [%s]' % self.get_err_yaw())
              
            self.change_state(0)
            rospy.loginfo('go to the goal')                                       
        else:
            self.follow_the_wall()
    self.count_loop += 1
    if self.count_loop > 20:
        self.count_time += 1
        self.count_loop = 0
