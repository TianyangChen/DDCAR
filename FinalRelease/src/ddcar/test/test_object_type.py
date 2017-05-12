#!/usr/bin/env python
# Author: Yuanzhengyu Li
# This test is for requirement A: recognize the type of barriers

PKG = 'ddcar'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import ast
import sys, time
import unittest
import rospy
import rostest
import xml.sax

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, Int32, String

NAME = 'test_object_type'

class TestOn_object_type(unittest.TestCase):
    def __init__(self, *args):
        super(TestOn_object_type, self).__init__(*args)



    # use this function to sort the expected_objects
    def key_x(self, dic):
        return dic['x']



    def test_object_type(self):
        global detected_objects
        rospy.init_node(NAME, anonymous=True)

        # wait for 30 seconds
        # we will begin this test after the world file has been generated
        timeout_t = time.time() + 30.0 
        while not rospy.is_shutdown() and time.time() < timeout_t:
            time.sleep(0.1)

        # fetch expected_objects and detected_objects variable
        expected_objects_string=rospy.get_param("expected_objects")
        detected_objects_string=rospy.get_param('/detected_objects')
        expected_objects=ast.literal_eval(expected_objects_string)
        detected_objects=ast.literal_eval(detected_objects_string)
        expected_objects_type=[]
        detected_objects_type=[]
        for x in sorted(expected_objects, key=self.key_x):
            expected_objects_type.append(x['type'])
        for x in detected_objects:
            detected_objects_type.append(x['type'][0])
        min_index=min(len(expected_objects_type), len(detected_objects_type))

        # compare the type of each objects
        correct_object_cnt=0.0
        for i in range(min_index):
            if expected_objects_type[i]==detected_objects_type[i]:
                correct_object_cnt+=1
        accuracy=correct_object_cnt/len(expected_objects)

        # if more than 75% of the objects are correct, this test is passed
        self.assertGreater(accuracy, 0.75)

if __name__ == '__main__': 
    rostest.rosrun(PKG, 'test_object_type', TestOn_object_type, sys.argv)