#!/usr/bin/env python
# Author: Tianyang Chen
# This test is for requirement A: position of the barriers

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

NAME = 'test_object_pose'

class TestOn_object_pose(unittest.TestCase):
    def __init__(self, *args):
        super(TestOn_object_pose, self).__init__(*args)



    # use this function to sort the expected objects array
    def key_x(self, dic):
        return dic['x']



    def test_object_pose(self):
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
        expected_objects_x=[]
        expected_objects_y=[]
        detected_objects_x=[]
        detected_objects_y=[]
        for x in sorted(expected_objects, key=self.key_x):
            expected_objects_x.append(x['x'])
            expected_objects_y.append(x['y'])
        for x in detected_objects:
            pose_string = x['type'][1]
            split_string=pose_string.split()
            detected_objects_x.append(float(split_string[0]))
            detected_objects_y.append(float(split_string[1]))
        min_index=min(len(expected_objects_x), len(detected_objects_x))

        # if the detected coordinate is within +/-2 of the expected coordinate, this object is considered as correct
        correct_object_cnt=0.0
        for i in range(min_index):
            if abs(detected_objects_x[i]-expected_objects_x[i])<2.0 and abs(detected_objects_y[i]-expected_objects_y[i])<2.0:
                correct_object_cnt+=1
        accuracy=correct_object_cnt/len(expected_objects)

        # if more than 75% of the objects are correct, this test is passed
        self.assertGreater(accuracy, 0.75)



if __name__ == '__main__': 
    rostest.rosrun(PKG, 'test_object_pose', TestOn_object_pose, sys.argv)