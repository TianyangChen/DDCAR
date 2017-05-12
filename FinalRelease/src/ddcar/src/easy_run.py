#!/usr/bin/env python

# Author: Tianyang Chen, Yawei Ding, Yuanzhengyu Li
# Copyright (c) 2016-2017 Arizona Board of Regents
# All rights reserved
#
# Permission is hereby granted, without written agreement and without 
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that the 
# above copyright notice and the following two paragraphs appear in 
# all copies of this software.
# 
# IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY 
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES 
# ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN 
# IF THE ARIZONA BOARD OF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF 
# SUCH DAMAGE.
# 
# THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, 
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
# AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
# IS ON AN "AS IS" BASIS, AND THE ARIZONA BOARD OF REGENTS HAS NO OBLIGATION
# TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

# What does this file do?
# THE purpose of this file is to control the velocity of the vehicle.
# This node will measure the distance between the vehicle and the barrier in front of the vehicle using the front_laser_points on the car.
# It controls the vehicle with 3 status: move backword, move forward and turn left

import rospy
import math
from math import sin, cos, sqrt
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Polygon, PolygonStamped, Point, Twist, Vector3
from nav_msgs.msg import Odometry

# initialize some global variables
cat_pose=Point()
cat_pose.x=0
cat_pose.y=0
cat_pose.z=0
now_time=rospy.Time()
init_time=rospy.Time()
only_once=0





def callback(data):
    global cat_pose
    global now_time
    global init_time
    global only_once

    #init some important parameters
    if only_once==0:
        init_time=rospy.Time.now()
        only_once=1
        pass
    now_time = rospy.Time.now()
    view=[]
    raw_data=data.ranges
    x=[]
    y=[]

    # fetch the data points that in front of the vehicle, and compute the minimum
    num_points = len(data.ranges)
    pi = 3.1415926
    for i in range(num_points):
        x.append(raw_data[i]*cos(i*pi/num_points-pi/2)+2.4)
        y.append(raw_data[i]*sin(i*pi/num_points-pi/2))
        pass
    for i in range(num_points):
        if y[i]>-1.2 and y[i]<1.2:
            view.append(x[i])
            pass
        pass

    # initialize the message we are going to publish
    pub = rospy.Publisher('catvehicle/cmd_vel', Twist, queue_size=10)
    msg=Twist()
    v_linear=Vector3()
    v_angular=Vector3()

    # two constrains of the vehicle
    # 1. the vehicle can only move within the circle of radius 30
    # 2. the vehicle can only move within 25 seconds
    if sqrt(cat_pose.x * cat_pose.x + cat_pose.y * cat_pose.y) < 30 and now_time - init_time < rospy.Duration(25):
        print min(view)
        if min(view)<4: # if the distance is less than 4, the vehicle will move backword
            v_linear.x=-2
            v_linear.y=0
            v_linear.z=0
            v_angular.x=0
            v_angular.y=0
            v_angular.z=0
            msg.linear=v_linear
            msg.angular=v_angular
            pub.publish(msg)
        elif min(view)<10: # if the distance is less than 10 ,turn left
            v_linear.x=1.2
            v_linear.y=0
            v_linear.z=0
            v_angular.x=0
            v_angular.y=0
            v_angular.z=0.5
            msg.linear=v_linear
            msg.angular=v_angular
            pub.publish(msg)
        else:
            v_linear.x=1.2 # if the distance is greater than 10, go straight
            v_linear.y=0
            v_linear.z=0
            v_angular.x=0
            v_angular.y=0
            v_angular.z=0
            msg.linear=v_linear
            msg.angular=v_angular
            pub.publish(msg)
        pass
    else: # if the vehicle doesn't meet any one of the constrains, we will control the vehicle to stop 
        v_linear.x=0
        v_linear.y=0
        v_linear.z=0
        v_angular.x=0
        v_angular.y=0
        v_angular.z=0
        msg.linear=v_linear
        msg.angular=v_angular
        pub.publish(msg)





def callback2(data2):
	# this function update the global coordinate of the vehicle according to "/catvehicle/odom"
    global cat_pose
    cat_pose.x = data2.pose.pose.position.x
    cat_pose.y = data2.pose.pose.position.y





def easy_run():

    # Initialize our node, which we will call easy_run
    rospy.init_node('easy_run', anonymous=True)
    global now_time
    global init_time
    
    # Create the subscriber, subscribe to topic "/catvehicle/front_laser_points" and "/catvehicle/odom"
    sub = rospy.Subscriber("catvehicle/front_laser_points", LaserScan, callback)   
    sub2 = rospy.Subscriber("catvehicle/odom", Odometry, callback2)
    
    rospy.spin()





if __name__ == '__main__':
    try:
        easy_run()
    except rospy.ROSInterruptException:
        pass

