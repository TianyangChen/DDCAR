#!/usr/bin/env python

# Author: Tianyang Chen
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
# THE purpose of this file is to record all the data from "/catvehicle/front_laser_points", 
# and use these data to identify the type and position of these objects.
# In the end, it will generate a world file.


import rospy
import math
import cv2
import ast
import numpy as np
from math import sin, cos, floor, ceil, acos
from geometry_msgs.msg import Polygon, PolygonStamped, Point, Point32
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Bool, Int32
from nav_msgs.msg import Odometry
from PIL import Image

# initialize some global variables
draw_cnt=0
img_matrix=np.zeros((2401,2401),dtype=np.uint8)
cat_pose=Point()
cat_pose.x=0
cat_pose.y=0
cat_pose.z=0
theta=0
obj_cnt=0
dic=[]





# this function analyse and store the front_laser_points data
def record_points(data):
    global img_matrix
    global cat_pose
    global theta
    count=0
    num_points = len(data.ranges)

    pi = 3.1415926
    for r in data.ranges:
        count+=1
        if r>=data.range_min and r<=data.range_max:
            # convert the laser data to Cartesian coordinates
            x=r*cos(count*pi/num_points-pi/2)+2.4
            y=r*sin(count*pi/num_points-pi/2)

            # convert laser data to global coordinates
            x_global=cat_pose.x+x*cos(theta)-y*sin(theta)      
            y_global=cat_pose.y+x*sin(theta)+y*cos(theta)

            # store the laser data in the image matrix
            x_global=int(round(x_global*10))
            y_global=int(round(y_global*10))

            # we only use the data that is within 20 unit distance
            if r<=20 and abs(x_global)<=1200 and abs(y_global)<=1200:
                img_matrix[1200- x_global,1200- y_global]=255

    



# this function export the raw data of laser points
# you can find this image in ~/.ros/point_map.png
def draw_now(img_matrix):
    img=Image.fromarray(img_matrix)
    img.save('point_map.png')





# this function generate the world file in ~/.ros
# the input variable "worldarray" is an array of python dict
# the struct of worldarray:
# [{'type':['model_name', 'pose']}, {'type':['model_name', 'pose']}, {'type':['model_name', 'pose']}, ...]
def writeworldfile(worldarray):
    fo = open("gazebo_output.world", "wb")
    fo.write( "<?xml version=\"1.0\" ?>\n")
    fo.write( "<sdf version=\"1.4\">\n<world name=\"default\">\n")
    fo.write("<include>\n<uri>model://ground_plane</uri>\n</include>\n<include>\n<uri>model://sun</uri>\n</include>\n")
    for alldict in worldarray:
        fo.write( "<include>\n")
        fo.write("<uri>model://")
        fo.write(alldict['type'][0])
        fo.write("</uri>\n")
        fo.write("<pose>")
        fo.write(alldict['type'][1])
        fo.write("</pose>\n")
        fo.write( "</include>\n")
    fo.write("</world>\n")
    fo.write("</sdf>\n")
    fo.close()





# this function analyse the laser data
def img_analysis(img_matrix):
    global obj_cnt
    global dic
    print "begin img_analysis"

    # using OpenCV
    # dilate the image with a 3*3 square
    kernel = np.ones((3,3), np.uint8)
    dilation = cv2.dilate(img_matrix, kernel, iterations = 1)

    # export the image after dilation
    # you can find this image in ~/.ros/dilation.png
    cv2.imwrite("dilation.png", dilation)

    # use the contour() funtion from OpenCV to do image segmentation
    contours, hierarchy = cv2.findContours(dilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) 
    contours_cnt=len(contours)
    x=[]
    y=[]
    x_center=0
    y_center=0
    dic=[]
    for i in range(contours_cnt):
        x=[]
        y=[]
        for j in contours[i]:
            x.append(j[0][0])
            y.append(j[0][1])
            pass
        # use bounding box to compute the center point of the object
        x_center=(1200-(max(x)+min(x))/2.0)/10
        y_center=(1200-(max(y)+min(y))/2.0)/10
        x_interval=max(x)-min(x)
        y_interval=max(y)-min(y)
        
        # use bounding box to detect the type of the object
        if x_interval>50 or y_interval>50:
            dic.append({'type':['house_3',str(y_center)+' '+str(x_center)+' 0.0 0.0 0.0 0.0']})
            pass
        elif x_interval>40 and y_interval<10:
            dic.append({'type':['jersey_barrier',str(y_center)+' '+str(x_center)+' 0.0 0.0 0.0 1.57']})
        elif x_interval<10 and y_interval>40:
            dic.append({'type':['jersey_barrier',str(y_center)+' '+str(x_center)+' 0.0 0.0 0.0 0.0']})
        elif x_interval>10 and y_interval>10:
            dic.append({'type':['construction_cone',str(y_center)+' '+str(x_center)+' 0.0 0.0 0.0 0.0']})
        else:
            pass

    # set the number of barriers we detected
    obj_cnt=len(dic)

    # set this variable "dic" to ROS Parameter Server
    rospy.set_param('detected_objects', str(dic))

    # generate world file
    writeworldfile(dic)

    # User interface output
    print "Generate world file successfully"
    print "finish analysis"





def start_draw(stop_status):
    global draw_cnt
    global img_matrix
    global probability_matrix

    # user interface
    # If the vehicle is moving, it will display "waiting the vehicle to stop"
    # If the vehicle has stopped and finished generating world file, it will display "You can shut down this launch file now"
    if draw_cnt==0:
        print "waiting the vehicle to stop"
    else:
        print "You can shut down this launch file now"
    
    # analyse the laser data
    if stop_status.data==True and draw_cnt==0:
        print "I begin to draw"
        # generate two images in ~/.ros
        draw_now(img_matrix)
        draw_cnt=1
        # analyse the image and generate world file
        img_analysis(img_matrix)
        pass





def pub_barrier_cnt(obj_cnt):
    # publish the number of barriers we detected
    pub = rospy.Publisher('barrier_cnt', Int32, queue_size=100)
    pub.publish(obj_cnt) 





def update_coordinate(data2):
    global cat_pose
    global theta
    global obj_cnt
    global img_matrix

    # update the global coordinate and steering angle of the vehicle, 
    # we will use these variables to update the front_laser_points data 
    cat_pose.x = data2.pose.pose.position.x
    cat_pose.y = data2.pose.pose.position.y
    theta = 2*acos(data2.pose.pose.orientation.w)

    # publish the number of barriers we detected
    pub_barrier_cnt(obj_cnt)





def main():
    # Initialize our node, which we will call draw_img
    rospy.init_node('draw_img', anonymous=True)

    # subscribe to topic "/catvehicle/front_laser_points"
    sub = rospy.Subscriber("catvehicle/front_laser_points", LaserScan, record_points)

    # subscribe to topic "/doIt", this topic publishes the data which represent the status of the vehicle
    # True denotes the vehicle has stopped, we will begin analysing the data
    sub_stop_status = rospy.Subscriber("doIt", Bool, start_draw)

    # subscribe to topic "/catvehicle/odom"
    sub_odom = rospy.Subscriber("catvehicle/odom", Odometry, update_coordinate)
        
    rospy.spin()





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

