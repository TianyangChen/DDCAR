#!/usr/bin/env python
# Author: Yawei Ding
# This test is for requirement B: number of barriers as expected

PKG = 'ddcar'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys, time
import unittest
import rospy
import rostest
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, Int32, String

NAME = 'barrier_cnt'

class TestOn_number_of_barriers(unittest.TestCase):
    def __init__(self, *args):
        super(TestOn_number_of_barriers, self).__init__(*args)
        self.barrier_cnt = 0



    def callback(self, data):
        self.barrier_cnt = data.data


           
    def test_number_of_barriers(self):
        # subscribe to topic "/barrier_cnt"
        rospy.Subscriber("barrier_cnt", Int32, self.callback)
        rospy.init_node(NAME, anonymous=True)

        # wait for 30 seconds
        timeout_t = time.time() + 30.0 
        while not rospy.is_shutdown() and time.time() < timeout_t:
            time.sleep(0.1)

        # test whether the number of detected barriers is the same as expected
        expected_barrier_num=rospy.get_param("expected_barrier_num")
        self.assertEqual(self.barrier_cnt,expected_barrier_num)



if __name__ == '__main__': 
    rostest.rosrun(PKG, 'test_number_of_barriers', TestOn_number_of_barriers, sys.argv)