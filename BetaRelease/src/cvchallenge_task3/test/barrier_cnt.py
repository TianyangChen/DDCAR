#!/usr/bin/env python
PKG = 'cvchallenge_task3'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys, time
import unittest
import rospy
import rostest
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, Int32, String

NAME = 'test_barriers_cnt'
class TestOn_number_of_barriers(unittest.TestCase):
    def __init__(self, *args):
        super(TestOn_number_of_barriers, self).__init__(*args)
        self.barrier_cnt = 0
        
    def callback(self, data):
        # print(rospy.get_caller_id(), "I heard %s" % data.data)
        #greetings is only sent over peer_publish callback, so hearing it is a success condition
        # if data.linear.x==0:
        self.barrier_cnt = data.data

    def test_number_of_barriers(self):
        rospy.Subscriber("barrier_cnt", Int32, self.callback)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + 30.0*1000 #10 seconds
        while not rospy.is_shutdown() and time.time() < timeout_t:
            time.sleep(0.1)
        # self.assert_(self.success, str(self.success))
        self.assertEqual(self.barrier_cnt,10)

if __name__ == '__main__': 
    # rostest.rosrun(PKG, 'test_name', 'MyTestSuite')
    rostest.rosrun(PKG, 'test_number_of_barriers', TestOn_number_of_barriers, sys.argv)