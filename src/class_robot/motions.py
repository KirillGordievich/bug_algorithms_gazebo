import rospy
import math

def go_to_point(self, point):
        d = self.safe_distance
        min_d = self.min_safe_distance
        self.state_description = 'case 0 - go to point'
	x_goal, y_goal = point[0], point[1]
        pose = self.get_pose()
        distance = self.get_distance(x_goal, y_goal, pose[0], pose[1])
        err_yaw = self.get_err_yaw()
        # stop if the agent is close to the goal
        if distance < self.distance_precision:
           self.yaw_past_error = 0
           linear, angular = 0, 0
           self.distance_past_error = distance
           rospy.loginfo('Current position = [%s, %s],  the goal: [%s, %s]' % (pose[0], pose[1], x_goal, y_goal))
           rospy.signal_shutdown("The goal is reached")
        # robot heading differs from the desired heading by more than the yaw precision
        elif math.fabs(err_yaw) < self.yaw_precision:
            # linear = K*error + D*(present_error - past_error)
            angular = 0
            linear = self.k_p*(distance) + self.k_d*abs(distance-self.distance_past_error)
            self.yaw_past_error = 0
            self.distance_past_error = distance
	else:
            # angular = K*error + D*(present_error - past_error)
            # linear = K*error + D*(present_error - past_error)
            # PD controller does not work perfectly
	    angular = -(err_yaw*self.k_p + self.k_d*abs(err_yaw-self.yaw_past_error))
            linear = self.k_p*(distance) + self.k_d*abs(distance-self.distance_past_error)
            self.yaw_past_error = err_yaw
            self.distance_past_error = distance
        # speed must not be higher than maximum speed
        if abs(linear) > self.max_linear_velocity:
            linear = self.max_linear_velocity
        if abs(angular) > self.max_angular_velocity:
            angular = self.max_angular_velocity
        self.set_velocity([linear, angular])

def follow_the_wall(self):
        if self.turn_left_mode is True:
            coef = 1
        else:
            coef = -1
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
                self.set_velocity([0.3, coef*(-0.2)])
            else:
                self.set_velocity([0.3, coef*0.4])
        if self.state == 2: # 'turn rigth or left
            if self.regions['front'] < min_d:
                self.set_velocity([0.2, coef*(-0.5)])
            else:
                self.set_velocity([0.3, coef*(-0.4)])
        if self.state == 3: # 'follow the wall'
            if self.regions['fleft']  < min_d or self.regions['left'] < min_d:
                self.set_velocity([0.4, coef*(-0.6)])
            else:
                self.set_velocity([0.4, coef*(0.0)])

