#!/usr/bin/python

import rospy
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from turtlesim.msg import Pose


class Robot:
    'class robot'
    from bug_1 import bug_1
    from bug_2 import bug_2
    from callback import callback_pose, callback_laser
    from motions import go_to_point, follow_the_wall
    def __init__(self):
        self.name = 'two_wheeled_robot' # name of the robot (string)
        self.status = "initialization" # Initialization, wait or move

        self.rate_hz = 20 # Refresh Rate of the main method

        self.turn_left_mode = True # False if you want to turn rigth

        self.state = 0 # 0, 1, 2 or 3 (int), 0 = go to point, 1,2,3 = wall follower
        self.state_description = 'start' # short descriprion of the state
        self.state_dict = {0: 'go to point', 1: 'find the wall', 2: 'turn left', 3: 'follow the wall'}

        self.count_time = 0 # using count_time to count seconds the robot is in a state 
        self.count_loop = 0 # using in timer 
  
        self.closest_point = None # for bug1
        self.starting_point = None # for bug1
        self.bug1_steps = {'go to point': True, 'circumnavigate obstacle': False, 'go to closest point': False}
        
        self.goal = [6.1, 1.5] # x and y coordinates of the goal 
        self.initial_position = None # using in bug2 
        self.x = 0.0 # x coordinate of the robot
        self.y = 0.0 # y coordinate of the robot
        self.yaw = 0.0 # oriention of the robot
        # velocity of the robot
	self.linear_velocity = 0 
	self.angular_velocity = 0 

	self.safe_distance = 1.3  # safe distance beetwen the robot and obstacles
        self.min_safe_distance = 0.8  # min safe distance beetwen robot and obstacles
        self.max_linear_velocity = 0.4 # max linear velocity of the robot
        self.max_angular_velocity = 1.2  # max angular velocity of the robot
        self.distance_precision = 0.25 # if we approach the target at a distance less than this value, we should stop
        self.yaw_precision = math.pi / 45 # +/- 2 degree allowed
        self.max_angle_error = 0.06 # in radians 
 
        # for PD controller
        self.yaw_past_error = 0 
        self.yaw_present_error = 0
        self.distance_past_error = 0      
        self.distance_present_error = 0 
        self.k_p = 0.5 # the coefficient for the proportional terms
        self.k_d = 0.2 # the coefficient for the derivative term

        # this dictionary contain distance to the closest obstacle 
        # in different regions around the robot (front, right, etc)
        self.regions = {'right': None, 'fright': None, 'front': None, 'fleft': None, 'left': None}
        # velocity msgs
        self.vel_msg = Twist()
        # register pub to send twist velocity 
        self.velocity_pub = rospy.Publisher("/" + self.name + "/cmd_vel", Twist, 
                                            queue_size=10)
        # register sub to get the robot pose
        self.pose_subscriber = rospy.Subscriber("/" + self.name + '/pose', Pose, 
                                                self.callback_pose, queue_size=6)
        # register sub to get the laser data of the robot
        self.pose_subscriber = rospy.Subscriber("/" + self.name + '/laser/scan', 
                                                LaserScan, self.callback_laser)
        # do some cleanup on shutdown
        rospy.on_shutdown(self.clean_shutdown)

    def main(self):
        while None in self.regions.values(): # waint until receive laser data
            rospy.sleep(2) # waint 2 sec
        rospy.loginfo('laser data is received, go')
        self.change_status('move') # change from initialization to move 
        rospy.loginfo('Initialization is done')
        rospy.loginfo('Initial position of the robot = %s' % self.initial_position)

        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.status == 'move':
                # turn on bug 1 algorithm
                self.bug_1()
            else:
                rospy.loginfo('unknown status')
            linear, angular = self.get_velocity()
            self.vel_msg.linear.x = linear
            self.vel_msg.angular.z = angular
            self.velocity_pub.publish(self.vel_msg)

            if not rospy.is_shutdown():
                rate.sleep()

    def can_head_toward_goal(self):
        # return True if the robot can head toward the goal or False if it cannot
        err_yaw = self.get_err_yaw()

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

    def get_err_yaw(self): 
        goal_x, goal_y = self.get_goal()
        pose_x, pose_y = self.get_pose()
        desired_yaw = math.atan2(goal_y - pose_y, goal_x - pose_x)
        err_yaw = desired_yaw - self.yaw
        return err_yaw

    def get_goal(self):
	return self.goal

    def get_distance(self, x1, y1, x2, y2):
        # the distance between two points (X1, y1) and (x2, y2)
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def get_yaw(self):
        # get yaw of the robot 
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
        # return the pose of the bot
        return [self.x, self.y]

    def clean_shutdown(self):
        # Stop robot when shutting down 
        rospy.loginfo('System is shutting down. Stopping %s...' % (self.name) )
        # stop the robot if the system is shutting down
        linear, angular  = 0, 0
        self.vel_msg.linear.x = linear
        self.vel_msg.angular.z = angular
        self.velocity_pub.publish(self.vel_msg)
        rospy.loginfo("Stop")

