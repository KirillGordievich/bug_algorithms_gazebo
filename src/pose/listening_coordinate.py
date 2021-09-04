#!/usr/bin/python

import rospy
from turtlesim.msg import Pose
from tf.transformations import euler_from_quaternion
import math
import re
from tf.msg import *

rospy.init_node("robot_pose")

def CallBack(data):
    msg = Pose()
    for i in range(len(data.transforms)): #data.transforms is a list 
     msg.x = data.transforms[i].transform.translation.x 
     msg.y = data.transforms[i].transform.translation.y
     euler = euler_from_quaternion([data.transforms[i].transform.rotation.x, data.transforms[i].transform.rotation.y, data.transforms[i].transform.rotation.z, data.transforms[i].transform.rotation.w])
     msg.theta = euler[2]
     p = rospy.Publisher("/two_wheeled_robot/pose", Pose, queue_size=2)
     p.publish(msg)

def listener():
    s = rospy.Subscriber("/tf", tfMessage, CallBack)
    rospy.spin()


if __name__ == '__main__':
     listener()
