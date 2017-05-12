#!/usr/bin/env python
# Author: Yuanzhengyu Li
# This test is for requirement B: automatically stop

PKG = 'ddcar'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys, time
import unittest
import rospy
import rostest
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

NAME = 'test_vehicle_stop'

class TestOn_Vehicle_Stop(unittest.TestCase):
    def __init__(self, *args):
        super(TestOn_Vehicle_Stop, self).__init__(*args)
        self.v = 0
        
    def callback(self, data):
        # update linear velocity
        self.v = data.linear.x

    def test_vehicle_stop(self):
        # subscribe to topic "/catvehicle/vel"
        rospy.Subscriber("catvehicle/vel", Twist, self.callback)
        rospy.init_node(NAME, anonymous=True)
        # wait for 30 seconds
        timeout_t = time.time() + 30.0 
        while not rospy.is_shutdown() and time.time() < timeout_t:
            time.sleep(0.1)
        # if the linear velocity is almost equal to 0, this test is passed
        self.assertAlmostEqual(self.v, 0, 2)

if __name__ == '__main__': 
    rostest.rosrun(PKG, 'test_vehicle_stop', TestOn_Vehicle_Stop, sys.argv)

    
    