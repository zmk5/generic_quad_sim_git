#!/usr/bin/env python

import numpy as np
import rospy

import cv2
import sys
from threading import Thread, Lock
import time
from std_msgs.msg import Float32, Empty
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, Wrench, Pose
from nav_msgs.msg import Odometry
import math
from cv_bridge import CvBridge, CvBridgeError


K_MOTOR = 2.980e-6
B_MOTOR = 1.140e-7

class QuadrotorController():
    def __init__(self):
        self.force[4] = 0.0
        self.ang_vel[4] = 0.0
        self.done = False
        self.running = True

        self.cvbridge = CvBridge()
        self.imu_msg = None
        self.imu_subs = rospy.Subscriber('/generic_quad')



