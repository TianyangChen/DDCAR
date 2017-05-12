#!/usr/bin/env python
PKG = 'cvchallenge_task3'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys, time
import unittest
import rospy
import rostest
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

NAME = 'test_vehicle_stop_finally'
class TestOn_Vehicle_Stop(unittest.TestCase):
    def __init__(self, *args):
        super(TestOn_Vehicle_Stop, self).__init__(*args)
        self.v = 0
        
    def callback(self, data):
        # print(rospy.get_caller_id(), "I heard %s" % data.data)
        #greetings is only sent over peer_publish callback, so hearing it is a success condition
        # if data.linear.x==0:
        self.v = data.linear.x

    def test_vehicle_stop(self):
        rospy.Subscriber("catvehicle/vel", Twist, self.callback)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + 30.0*1000 #10 seconds
        while not rospy.is_shutdown() and time.time() < timeout_t:
            time.sleep(0.1)
        # self.assert_(self.success, str(self.success))
        self.assertAlmostEqual(self.v, 0, 2)

# class TestOn_Vehicle_Safty(unittest.TestCase):
#     def __init__(self, *args):
#         super(TestOn_Vehicle_Safty, self).__init__(*args)
#         self.range_min = 80
        
#     def callback(self, data):
#         # print(rospy.get_caller_id(), "I heard %s" % data.data)
#         #greetings is only sent over peer_publish callback, so hearing it is a success condition
#         # if data.linear.x==0:
#         self.range_min = min(data.ranges)

#     def test_vehicle_safty(self):
#         rospy.Subscriber("catvehicle/front_laser_points", LaserScan, self.callback)
#         #rospy.init_node('test_vehicle_safty_finally', anonymous=True)
#         #timeout_t = time.time() + 30.0*1000 #10 seconds
#         # while time.time() < timeout_t:
#         #     time.sleep(0.1)
#         # self.assert_(self.success, str(self.success))
#         self.assertGreater(self.range_min, 4)

# class MyTestSuite(unittest.TestSuite):

#     def __init__(self):
#         super(MyTestSuite, self).__init__()
#         self.addTest(TestOn_Vehicle_Stop())
#         self.addTest(TestOn_Vehicle_Safty())
if __name__ == '__main__': 
    # rostest.rosrun(PKG, 'test_name', 'MyTestSuite')
    rostest.rosrun(PKG, 'test_vehicle_stop', TestOn_Vehicle_Stop, sys.argv)

    
    