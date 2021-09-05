#!/usr/bin/python
import rospy
import math

from geometry_msgs.msg import Twist

def bug_2(self):
        d = self.safe_distance
        min_d = self.min_safe_distance
        epsilon = 0.3
        goal_x, goal_y = self.get_goal()
        pose_x, pose_y = self.get_pose()
        # 0 is the "go to point" state
        if self.state == 0: 
            # if robot meet an obstacle 
            if self.regions['front'] < d:
                rospy.loginfo('self.regions[front] = %s ' %self.regions['front'])
                self.count_time = 0
                self.count_loop = 0
                self.closest_point = self.get_pose()
                # if robot meet an obstacle we should circumnavigate it 
                self.bug1_steps['go to point'] = False
                self.bug1_steps['circumnavigate obstacle'] = True
                self.change_state(1)
            else:
                self.go_to_point(self.get_goal())
        # wall follower states
        elif self.state in [1,2,3]:
            # first step of bug 1:
            # if an obstacle is encountered, circumnavigate it 
            # and remember how close you get to the goal
            if self.bug1_steps['circumnavigate obstacle'] is True:
                if self.count_time < 1 and self.count_loop >= 17: # need some time to reach the obstacle to set starting_point 
                    self.starting_point = self.get_pose()
                    rospy.loginfo('Starting point = %s' % self.starting_point)               
                self.follow_the_wall()
                # if current position is closer to the goal than the previous closest_position, assign current position to closest_point
                 # get the x and y coordinate of the local goal
                current_dist = self.get_distance(goal_x, goal_y, pose_x, pose_y)
                min_dist = self.get_distance(goal_x, goal_y, self.closest_point[0], self.closest_point[1])
                if current_dist < min_dist:
                    self.closest_point = [pose_x, pose_y]
                # compare only after 4 seconds - need some time to get out of starting_point
                if (self.count_time > 4):
                    start_x, start_y = self.starting_point
                    dist_to_start_point = self.get_distance(start_x, start_y, pose_x, pose_y)
                    rospy.loginfo_throttle(1, 'dist_to_start_point = ' + str(dist_to_start_point))
                    # if robot reaches (is close to) starting point
                    if dist_to_start_point < epsilon:
                        rospy.loginfo('go to the closest point')
                        self.bug1_steps['circumnavigate obstacle'] = False
                        self.bug1_steps['go to closest point'] = True # go to closest point  
            # second step of bug 1
            # return to the closest point
            elif self.bug1_steps['go to closest point'] is True:
                dist_to_closest_point = self.get_distance(self.closest_point[0], self.closest_point[1], pose_x, pose_y)
                rospy.loginfo_throttle(1, 'distance to the closest point = ' + str(dist_to_closest_point))
                if dist_to_closest_point < epsilon:
                    # while robot heading differs from the desired heading by more than the yaw precision 
                    while math.fabs(self.get_err_yaw()) > self.yaw_precision:
                        linear, angular = 0, -0.25
                        self.vel_msg.linear.x = linear
                        self.vel_msg.angular.z = angular
                        self.velocity_pub.publish(self.vel_msg)
                        rospy.loginfo_throttle(1,'Yaw error: [%s]' % self.get_err_yaw())
                    self.change_state(0)
                    rospy.loginfo('go to the goal')
                    self.bug1_steps['go to point'] = True
                    self.bug1_steps['go to closest point'] = False
                else:
                    self.follow_the_wall()
        else:
             rospy.loginfo('unknown state or step')
        # timer
        self.count_loop += 1
        if self.count_loop > 20:
            self.count_time += 1
            self.count_loop = 0
        # print basic information 
        rospy.loginfo_throttle(1, 'state = ' + self.state_dict[self.state] + ', (%s)'% self.state)
        rospy.loginfo_throttle(1, 'State description = %s' % self.state_description)

