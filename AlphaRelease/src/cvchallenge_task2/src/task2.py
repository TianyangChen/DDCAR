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
# THE purpose of this file is to create and publish polygon ros messages
# this file only uses the front_laser_points on the car
# and be able to identify objects within 10 meters of the sensor


import rospy
import math
from math import sin, cos, floor, ceil
from geometry_msgs.msg import Polygon, PolygonStamped, Point, Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
cat_pose=Point()
cat_pose.x=0
cat_pose.y=0
cat_pose.z=0

def callback(data):
    #init some important parameters
    r = []
    count = 0
    angular = []
    sigma = 0.25 #this parameter denotes the sensitivity of the sensor
    obj_num = 1
    obj_num2 = 0
    seq = 0;
    num_points = len(data.ranges)
    r_index_divide = []
    pi = 3.1415926

    #filt all the points within 10 meters of the sensor
    rospy.loginfo(data)
    for x in data.ranges:
        count += 1
        if x < 10:
            r.append(x)
            angular.append(count)
            pass
        pass

    #this algorithm combine some adjacent points to the same object
    for r_index in range(len(r)-1):
        if r[r_index] - r[r_index+1] < -sigma or r[r_index] - r[r_index+1] > sigma:
            obj_num += 1
            pass
        pass

    #divide our laser data into different objects
    r_divide = [[] for i in range(obj_num)]
    angular_divide = [[] for i in range(obj_num)]
    r_divide[0].append(r[0])
    angular_divide[0].append(angular[0])
    for r_index in range(len(r)-1):
        if r[r_index] - r[r_index+1] >= -sigma and r[r_index] - r[r_index+1] <= sigma:
            r_divide[obj_num2].append(r[r_index+1])
            angular_divide[obj_num2].append(angular[r_index+1])
        else:
            obj_num2 += 1
            r_divide[obj_num2].append(r[r_index+1])
            angular_divide[obj_num2].append(angular[r_index+1])

    #publish all the polygons
    for i in range(obj_num):
        msg = PolygonStamped();
        detection = Polygon();
        t1 = rospy.Time.now();
        # puts a time stamp at t1 - as the initial point
        msg.header.stamp = t1;
        seq += 1;
        msg.header.seq = seq;
        # catvehicle/odom is the odometry topic you are subscribing to 
        msg.header.frame_id = 'catvehicle/odom'
        x = []; 
        y = []; 
        z = 0.1;
        #convert the polar coordinates to Catesian coordinates
        for j in range(len(r_divide[i])):
            x.append(r_divide[i][j]*sin(angular_divide[i][j]*pi/num_points)+2.4)

            y.append(-r_divide[i][j]*cos(angular_divide[i][j]*pi/num_points))
            pass
        # here we use the bounding box strategy to represent the barrier
        x_pose = cat_pose.x
        y_pose = cat_pose.y 
        detection.points.append(Point(min(x)+x_pose,min(y)+y_pose,z));
        detection.points.append(Point(max(x)+x_pose,min(y)+y_pose,z));
        detection.points.append(Point(max(x)+x_pose,max(y)+y_pose,z));
        detection.points.append(Point(min(x)+x_pose,max(y)+y_pose,z));
        # then this polygon is then converted into a polgon message
        msg.polygon = detection;
        # writing loginto into stdout
        rospy.loginfo(msg);
        # publishing the ros message with the polygon information
        pub = rospy.Publisher('detections', PolygonStamped, queue_size=10)
        rate = rospy.Rate(120)
        pub.publish(msg)

        # pub2 = rospy.Publisher('catvehicle/cmd_vel', Twist, queue_size=10)
        # # rate = rospy.Rate(1)
        # msg2=Twist()
        # v_linear=Vector3()
        # v_angular=Vector3()
        # v_linear.x=2
        # v_linear.y=0
        # v_linear.z=0
        # v_angular.x=0
        # v_angular.y=0
        # v_angular.z=0
        # msg2.linear=v_linear
        # msg2.angular=v_angular
        # rospy.loginfo(msg2)
        # pub2.publish(msg2)

        rate.sleep()
        pass


def callback2(data2):
    cat_pose.x = data2.pose.pose.position.x
    cat_pose.y = data2.pose.pose.position.y


def barrier_dect():
    # Initialize our node, which we will call task2
    rospy.init_node('task2', anonymous=True)

    # Create the subscriber, subscribe to topic /catvehicle/front_laser_points
    sub = rospy.Subscriber("catvehicle/front_laser_points", LaserScan, callback)

    sub2 = rospy.Subscriber("catvehicle/odom", Odometry, callback2)
    
    rospy.spin()

if __name__ == '__main__':
    barrier_dect()
    # try:
    #     main()
    # except rospy.ROSInterruptException:
    #     pass

