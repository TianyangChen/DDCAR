#!/usr/bin/env python
# Author: Yawei Ding
# This test is for requirement B: avoid all the barriers automatically while running

PKG = 'ddcar'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys, time
import unittest
import rospy
import rostest
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from math import sin, cos, floor, ceil, acos

NAME = 'test_vehicle_stop_finally'

class TestOn_Vehicle_Safty(unittest.TestCase):
    def __init__(self, *args):
        super(TestOn_Vehicle_Safty, self).__init__(*args)
        # this variablbe denotes the status of whether vehicle hit the barrier, True means not hit and False means hit
        self.not_hit = True
        


    def callback(self, data):
        view=[]
        raw_data=data.ranges
        x=[]
        y=[]
        num_points = len(data.ranges)
        pi = 3.1415926
        for i in range(num_points):
            x.append(raw_data[i]*cos(i*pi/num_points-pi/2)+2.4)
            y.append(raw_data[i]*sin(i*pi/num_points-pi/2))
            pass
        for i in range(num_points):
            if y[i]>-1.1 and y[i]<1.1:
                view.append(x[i])
                pass
            pass
        # the front laser is 2.4m from the center of the vehicle
        # for safty reasons, if the minimum distance is less than 3m, the vehicle is considered as hit
        if min(view)<3:
            self.not_hit=False
            pass



    def test_vehicle_safty(self):
        # subscribe to topic "/catvehicle/front_laser_points"
        rospy.Subscriber("catvehicle/front_laser_points", LaserScan, self.callback)
        rospy.init_node('test_safty', anonymous=True)

        # during 30 seconds, if at any time the vehicle hit, we will set not_hit to be False
        timeout_t = time.time() + 30.0 
        while not rospy.is_shutdown() and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self.not_hit)



if __name__ == '__main__': 
    rostest.rosrun(PKG, 'test_vehicle_safty', TestOn_Vehicle_Safty, sys.argv)
