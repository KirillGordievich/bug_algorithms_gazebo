#!/usr/bin/python

import rospy
import math
import random
import geometry_msgs.msg
import re
from sensor_msgs.msg import LaserScan
from turtlesim.msg import Pose


class Robot:
    def __init__(self):
        self.name = 'two_wheeled_robot' # name of the robot (string)
        self.status = "initialization" # Initialization, wait or move

        self.rate_hz = 20 # Refresh Rate of the main method

        self.state = 0
        self.state_description = 'start'
        self.state_dict = {1: 'find the wall', 2: 'turn left', 3: 'follow the wall', 0: 'go to point'}

        self.count_time = 0 # using count_time to count seconds the robot is in a state 
        self.count_loop = 0 # using in timer

        self.closest_point = None # for bug1
        self.starting_point = None # for bug1
        self.bug1_steps = {'go to point': True, 'circumnavigate obstacle': False, 'go to closest point': False}

        
        self.goal = [6.1, 1.5] # x and y coordinates of the goal 
        self.initial_position = [0, 0]
        self.x = 0.0 # x coordinate of the robot
        self.y = 0.0 # y coordinate of the robot
        self.yaw = 0.0 # oriention of the robot
        # velocity of the robot
	self.linear_velocity = 0 
	self.angular_velocity = 0 

	self.safe_distance = 1.3  # safe distance beetwen the robot and obstacles
        self.min_safe_distance = 0.8  # min safe distance beetwen robot and obstacles
        self.max_linear_velocity = 0.5 # max linear velocity of the robot
        self.max_angular_velocity = 2  # max angular velocity of the robot
        self.distance_precision = 0.25 # if we approach the target at a distance less than this value, we should stop 
        self.max_angle_error = 0.06 # in radians 
 
        # for PD controller
        self.angle_past_error = 0 
        self.angle_present_error = 0
        self.distance_past_error = 0      
        self.distance_present_error = 0 
        self.k_p = 0.5 # the coefficient for the proportional terms
        self.k_d = 0.2 # the coefficient for the derivative term

        self.regions = {'right': None, 'fright': None, 'front': None, 'fleft': None, 'left': None}

        self.geometry_msg = geometry_msgs.msg.Twist() # velocity msgs
        # register pub to send twist velocity 
        self.velocity_pub = rospy.Publisher("/" + self.name + "/cmd_vel", 
                                                  geometry_msgs.msg.Twist, queue_size=10)
        # register sub to get the robot pose
        self.pose_subscriber = rospy.Subscriber("/" + self.name + '/pose', 
                                                Pose, self.callback_pose, queue_size=6)
        # register sub to get the laser data of the robot
        self.pose_subscriber = rospy.Subscriber("/" + self.name + '/laser/scan', 
                                                LaserScan, self.callback_laser)
        # do some cleanup on shutdown
        rospy.on_shutdown(self.clean_shutdown)

    def main(self):
        while None in self.regions.values(): # waint until receive laser data
            rospy.sleep(1)
        rospy.loginfo('laser data is received, go')
        self.change_status('move')
        rospy.loginfo('Initialization is done')
        rospy.loginfo('Initial position of the robot = ' % self.initial_position)

        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.move()
            if not rospy.is_shutdown():
                rate.sleep()

    def move(self):
        if self.status == 'move':
            # turn on bug 1 algorithm
            self.bug_1()
        else:
            rospy.loginfo('unknown status')
        linear, angular = self.get_velocity()
        self.geometry_msg.linear.x = linear
        self.geometry_msg.angular.z = angular
        self.velocity_pub.publish(self.geometry_msg)

    def bug_1(self):
        d = self.safe_distance
        min_d = self.min_safe_distance
        epsilon = (d - min_d)/2
        goal_x, goal_y = self.get_goal()
        pose_x, pose_y = self.get_pose()

        if self.state == 0: # go to point
            # if meet an obstacle 
            if self.regions['front'] < d - epsilon: # REMOVE EPSILON! and try to move self.closest_point = self.get_pose() to the next condition
                self.count_time = 0
                self.closest_point = self.get_pose()
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
                if (self.count_time == 1): # need some time to reach the obstacle to set starting_point
                    self.starting_point = self.get_pose()               
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
                    while self.get_angle(goal_x, goal_x) > 1.5*self.max_angle_error:
                        linear, angular = 0, -0.1
                        self.geometry_msg.linear.x = linear
                        self.geometry_msg.angular.z = angular
                        self.velocity_pub.publish(self.geometry_msg)
                        rospy.loginfo('get_angle(goal_x, goal_x) = %s ' % self.get_angle(goal_x, goal_x))
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

    def go_to_point(self, point):
        d = self.safe_distance
        min_d = self.min_safe_distance
        self.state_description = 'case 0 - go to point'
	x_goal, y_goal = point[0], point[1]
        pose = self.get_pose()
        distance = self.get_distance(x_goal, y_goal, pose[0], pose[1])
        angle = self.get_angle(x_goal, y_goal)
        # stop if the agent is close to the goal
        if distance < self.distance_precision:
           self.angle_past_error = 0
           linear, angular = 0, 0
           self.distance_past_error = distance
           rospy.loginfo('Current position = [%s, %s],  the goal: [%s, %s]' % (pose[0], pose[1], x_goal, y_goal))
           rospy.signal_shutdown("The goal is reached")
        elif abs(angle) < self.max_angle_error:
            # linear = K*error + D*(present_error - past_error)
            angular = 0
            linear = self.k_p*(distance) + self.k_d*abs(distance-self.distance_past_error)
            self.angle_past_error = 0
            self.distance_past_error = distance
	else:
            # angular = K*error + D*(present_error - past_error)
            # linear = K*error + D*(present_error - past_error)
	    angular = -(angle*self.k_p + self.k_d*abs(angle-self.angle_past_error))
            linear = self.k_p*(distance) + self.k_d*abs(distance-self.distance_past_error)
            self.angle_past_error = angle
            self.distance_past_error = distance
        # speed must not be higher than maximum speed
        if abs(linear) > self.max_linear_velocity:
            linear = self.max_linear_velocity
        if abs(angular) > self.max_angular_velocity:
            angular = self.max_angular_velocity
        self.set_velocity([linear, angular])

    def follow_the_wall(self):
        left = True # False if you want to turn rigth
        if left is True:
            k = -1
        d = self.safe_distance
        min_d = self.min_safe_distance

        if (self.regions['front'] > d and self.regions['fleft'] > d and
           self.regions['fright'] > d):
            self.state_description = 'case 1 - nothing'
            self.change_state(1)
        elif (self.regions['front'] < d and self.regions['fleft'] > d and
             self.regions['fright'] > d):
            self.state_description = 'case 2 - front'
            self.change_state(2)
        elif (self.regions['front'] > d and self.regions['fleft'] > d and
             self.regions['fright'] < d):
            self.state_description = 'case 3 - fright'
            self.change_state(3)
        elif (self.regions['front'] > d and self.regions['fleft'] < d and 
             self.regions['fright'] > d):
            self.state_description = 'case 4 - fleft'
            self.change_state(1)
        elif (self.regions['front'] < d and self.regions['fleft'] > d and
             self.regions['fright'] < d):
            self.state_description = 'case 5 - front and fright'
            self.change_state(2)
        elif (self.regions['front'] < d and self.regions['fleft'] < d and
             self.regions['fright'] > d):
            self.state_description = 'case 6 - front and fleft'
            self.change_state(2)
        elif (self.regions['front'] < d and self.regions['fleft'] < d and
             self.regions['fright'] < d):
            self.state_description = 'case 7 - front and fleft and fright'
            self.change_state(2)
        elif (self.regions['front'] > d and self.regions['fleft'] < d and
             self.regions['fright'] < d):
            self.state_description = 'case 8 - fleft and fright'
            self.change_state(1)
        else:
            self.state_description = 'unknown case'

        if self.state == 1: # 'find the wall'
            if self.regions['fleft']  < min_d or self.regions['left'] < min_d:
                # WARNING! instead of just 0.3 and 0.2 need to add a new variable 
                self.set_velocity([0.3, k*0.2])
            else:
                self.set_velocity([0.3, k*-0.4])
        if self.state == 2: # 'turn rigth or left
            if self.regions['front'] < min_d:
                self.set_velocity([0.2, k*0.5])
            else:
                self.set_velocity([0.3, k*0.4])
        if self.state == 3: # 'follow the wall'
            if self.regions['fleft']  < min_d or self.regions['left'] < min_d:
                self.set_velocity([0.4, k*0.6])
            else:
                self.set_velocity([0.4, k*0.0])

    def can_head_toward_goal(self):
        # return True if the bot can head toward the goal or False if it cannot
        d = self.safe_distance
        goal_x, goal_y = self.get_goal()

        desired_yaw = math.atan2(goal_y - self.y, goal_x - self.x)
        yaw = self.yaw
        err_yaw = desired_yaw - yaw

        if (math.fabs(err_yaw) < (math.pi / 6) and 
            self.regions['front'] > d):
            return True
        elif (err_yaw > 0 and 
             math.fabs(err_yaw) > (math.pi / 4) and 
             math.fabs(err_yaw) < (math.pi / 2) and 
             self.regions['left'] > d):
            return True
        elif (err_yaw < 0 and 
             math.fabs(err_yaw) > (math.pi / 4) and 
             math.fabs(err_yaw) < (math.pi / 2) and 
             self.regions['right'] > d):
            return True
        else:
            return False

    def change_state(self, state): # go to point, turn rigth/left
        if state is not self.state: # if state == self.state there is no need to change the state
            self.state = state # change the state of the bot

    def change_status(self, status): # stope, move, init etc
        if status is not self.status: # if status == self.status there is no need to change the status
            self.status = status # change the status of the bot
 
    def set_velocity(self, velocity):
        self.linear_velocity, self.angular_velocity = velocity
 
    def set_status(self, data):
	self.status = data
        
    def get_velocity(self):
        return self.linear_velocity, self.angular_velocity  

    def get_goal(self): # need to add trade function
	return self.goal

    def get_distance(self, x1, y1, x2, y2):
        # the distance between two points (X1, y1) and (x2, y2)
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def get_yaw(self):
        # get yaw of the agent 
        yaw = self.yaw
        if self.yaw < 0:
            return (yaw+2*math.pi)  
        else:
            return self.yaw

    def get_distance_to_line(self, p0): # using in bug2
        # p0 is the current position
        # p1 and p2 points define the line
        p1 = self.initial_position # [x, y]
        p2 = self.goal # [x, y]
        # here goes the equation
        up_eq = math.fabs((p2[1] - p1[1]) * p0[0] - (p2[0] - p1[0]) * p0[1] + (p2[0] * p1[1]) - (p2[1] * p1[0]))
        lo_eq = math.sqrt(pow(p2[1] - p1[1], 2) + pow(p2[0] - p1[0], 2))
        distance = up_eq / lo_eq
        return distance

    def get_pose(self): 
        return [self.x, self.y] # return the pose of the bot
           
    def get_angle(self, x, y): # WARNING for some reason it works wrong sometime!!! 
        # get the angle between OX
        # and the line agent and goal points
        phi = math.atan2(y-self.y, x-self.x)
        yaw = self.get_yaw()
        if phi < 0: 
            phi += math.pi*2
        angle = self.get_error_angle(yaw, phi)
        return angle

    def get_error_angle(self, angle1, angle2):
        # get the real angle between two angle 
        return math.atan2(math.sin(angle2-angle1), math.cos(angle2-angle1))

    def callback_pose(self, data):
        self.x = data.x
        self.y = data.y 
        self.yaw = data.theta
        if self.status == 'initialization':
            self.initial_position = [data.x, data.y]

    def callback_laser(self, data): # split laser data into 5 regions
        self.regions = {
            'right':  min(min(data.ranges[0:143]), 10),
            'fright': min(min(data.ranges[144:287]), 10),
            'front':  min(min(data.ranges[288:431]), 10),
            'fleft':  min(min(data.ranges[432:575]), 10),
            'left':   min(min(data.ranges[576:719]), 10),
        }

    def clean_shutdown(self):
        ''' Stop robot when shutting down '''
        rospy.loginfo('System is shutting down. Stopping %s...' % (self.name) )
        # stop the robot if the system is shutting down
        linear, angular  = 0, 0
        self.geometry_msg.linear.x = linear
        self.geometry_msg.angular.z = angular
        self.velocity_pub.publish(self.geometry_msg)
        rospy.loginfo("Stop")

