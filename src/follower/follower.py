#! /usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np


class Init:
    def __init__(self):
        self.pose = Pose()
        self.sub_1 = rospy.Subscriber('/turtle1/pose', Pose, self.follow)
        self.sub_2 = rospy.Subscriber('/turtle2/pose', Pose, self.update)
        self.pub_2 = rospy.Publisher('/turtle2/cmd_vel', Twist)

    def update(self, my_pose):
        self.pose = my_pose

    def to_vector(self, pose):
        return np.array([pose.x, pose.y])

    def follow(self, target_pose):
        p1, p2 = self.to_vector(target_pose), self.to_vector(self.pose)
        msg = Twist()
        msg.linear.x = np.linalg.norm(p1 - p2)
        msg.angular.z = (np.arctan2(*reversed(p1 - p2)) - self.pose.theta) % (2 * np.pi)
        self.pub_2.publish(msg)


rospy.init_node('follower')
Init()
rospy.spin()
