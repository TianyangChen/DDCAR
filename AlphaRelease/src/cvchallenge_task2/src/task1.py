#!/usr/bin/env python

# Author: Jonathan Sprinkle
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
# THE purpose of this file is to create a publish a polygon ros message
# this file serves as a template as to how you will be able to use the sensors on the car
# and be able to identify objects in car's path in later tasks


import rospy
import math
from math import sin, cos, floor, ceil
from geometry_msgs.msg import Polygon, PolygonStamped, Point

def main():
    # Create the publisher, which is using the desired output topic and message
    pub = rospy.Publisher('detections', PolygonStamped, queue_size=10)
    # Initialize our node, which we will call task1
    rospy.init_node('task1', anonymous=True)
    # publish at a rate of 1Hz
    rate = rospy.Rate(1) # 1hz
    seq = 0;
    # a loop which iterates through as long as the ros server is online
    while not rospy.is_shutdown():
        # we use a sin function to make the object move around
        # polygon stamped is a type of ros message which allows you to be able to represent a polygon with a coordinate frame and time stamp
        msg = PolygonStamped();
        # polygon is a ros message which allows you to specify a polygon based on where the initial and final points are assumed to be connected
        detection = Polygon();
        # take the first point, and create a 'box' around it using rounding
        t1 = rospy.Time.now();
        # puts a time stamp at t1 - as the initial point
        msg.header.stamp = t1;
        # increment the sequence from last time
        seq += 1;
        msg.header.seq = seq;
        # catvehicle/odom is the odometry topic you are subscribing to 
        msg.header.frame_id = 'catvehicle/odom'
        # sinusoidal functions create a circular path and each point is then converted to a square
        # the generation of square is done below through ceil and floor functions of x and y points
        x = 3*math.sin(0.2*t1.to_sec()); #to_sec converts t1 into seconds
        y = 3*math.cos(0.2*t1.to_sec()); # creating a circle with respect to time
        z = 0.1;
        # appending 4 different x y z coordinates to basically build a polygon frame 
        detection.points.append(Point(floor(x),floor(y),z));
        detection.points.append(Point(ceil(x),floor(y),z));
        detection.points.append(Point(ceil(x),ceil(y),z));
        detection.points.append(Point(floor(x),ceil(y),z));
        # then this polygon is then converted into a polgon message
        msg.polygon = detection;
        # writing loginto into stdout
        rospy.loginfo(msg);
        # publishing the ros message with the polygon information
        pub.publish(msg);
        # rospy.rate is specified to be 1 hz so by doing rate.sleep() we are making ROS sleep for 1 second
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

