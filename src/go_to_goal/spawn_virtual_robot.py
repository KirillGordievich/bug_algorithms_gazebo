#!/usr/bin/python

import rospy
from class_bot import Robot


if __name__ == "__main__":
    rospy.init_node("~virtual_robot", anonymous=True)
    virtual_robot = Robot()
    virtual_robot.main()
    rospy.spin()

