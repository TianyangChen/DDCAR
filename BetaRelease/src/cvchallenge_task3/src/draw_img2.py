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
# THE purpose of this file is to create and publish polygon ros messages
# this file only uses the front_laser_points on the car
# and be able to identify objects within 10 meters of the sensor


import rospy
import math
import cv2

from math import sin, cos, floor, ceil, acos
from geometry_msgs.msg import Polygon, PolygonStamped, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, Int32, String
from nav_msgs.msg import Odometry

# import matplotlib.pyplot as plt
# import matplotlib.image as mpimg
import numpy as np
from PIL import Image
draw_cnt=0
# now_time=rospy.Time()
# init_time=rospy.Time()
img_matrix=np.zeros((1601,1601),dtype=np.uint8)
cat_pose=Point()
cat_pose.x=0
cat_pose.y=0
cat_pose.z=0
theta=0
obj_cnt=0

def record_points(data):
    # global now_time
    global img_matrix
    # global draw_cnt
    global cat_pose
    count=0
    num_points = len(data.ranges)
    pi = 3.1415926
    for r in data.ranges:
        count+=1
        x=r*cos(count*pi/num_points-pi/2)+2.4
        y=r*sin(count*pi/num_points-pi/2)
        x_global=cat_pose.x+x*cos(theta)-y*sin(theta)
        y_global=cat_pose.y+x*sin(theta)+y*cos(theta)
        x_global=int(round(x_global*10))
        y_global=int(round(y_global*10))
        if r<=20 and abs(x_global)<=800 and abs(y_global)<=800:
            img_matrix[800- x_global,800- y_global]=255
        pass
    # now_time=rospy.Time.now()
    # if now_time-init_time>rospy.Duration(10) and draw_cnt==0:
    #     draw_now(img_matrix)
    #     draw_cnt+=1
    #     pass
    



def draw_now(img_matrix):

    img=Image.fromarray(img_matrix)
    img.save('point_map.png')

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
        # fo.write("<name>")
        # fo.write(alldict['type'][0])
        # fo.write("</name>\n")
        fo.write("<pose>")
        fo.write(alldict['type'][1])
        fo.write("</pose>\n")
        fo.write( "</include>\n")
    fo.write("</world>\n")
    fo.write("</sdf>\n")
    fo.close()

# dict={'type':['house_1',"0.0 ,1.0,0.0,0.0, 0.0,0.0"]}
def img_analysis(img_matrix):
    global obj_cnt
    print "begin img_analysis"
    # img=cv2.imread('~/.ros/point_map.png')
    # img_bi=cv2.threshold(img_matrix,127,255,cv2.THRESH_BINARY)
    kernel = np.ones((5,5), np.uint8)
    dilation = cv2.dilate(img_matrix, kernel, iterations = 1)
    # erosion = cv2.erode(dilation, kernel, iterations = 1)
    # closing = cv2.morphologyEx(img_matrixm, cv2.MORPH_CLOSE, kernel)
    # opening = cv2.morphologyEx(img_matrix, cv2.MORPH_OPEN, kernel)
    # closing_opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
    cv2.imwrite("dilation.png", dilation)
    # cv2.imwrite("erosion.png", erosion)
    # cv2.imwrite("closing.png", closing)
    # cv2.imwrite("opening.png", opening)
    # cv2.imwrite("closing_opening.png", closing_opening)
    # ret, binary = cv2.threshold(closing,127,255,cv2.THRESH_BINARY)
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
        # contour=contours[i]
        # x.append(contours[i][0])
        # y.append(contours[i][1])
        x_center=(800-(max(x)+min(x))/2.0)/10
        y_center=(800-(max(y)+min(y))/2.0)/10
        x_interval=max(x)-min(x)
        y_interval=max(y)-min(y)
        if x_interval>50 or y_interval>50:
            dic.append({'type':['house_3',str(y_center)+' '+str(x_center)+' 0.0 0.0 0.0 0.0']})
            obj_cnt+=1
            pass
        elif x_interval>40 and y_interval<10:
            dic.append({'type':['jersey_barrier',str(y_center)+' '+str(x_center)+' 0.0 0.0 0.0 1.57']})
            obj_cnt+=1
        elif x_interval<10 and y_interval>40:
            dic.append({'type':['jersey_barrier',str(y_center)+' '+str(x_center)+' 0.0 0.0 0.0 0.0']})
            obj_cnt+=1
        elif x_interval>10 and y_interval>10:
            dic.append({'type':['construction_cone',str(y_center)+' '+str(x_center)+' 0.0 0.0 0.0 0.0']})
            obj_cnt+=1
        else:
            pass


    writeworldfile(dic)
    obj_cnt=len(dic)
    # rospy.loginfo("Generate world file successfully")
    print "Generate world file successfully"
    # draw_contour = cv2.drawContours(closing,contours,-1,(0,0,255),3)  
    # cv2.imwrite("draw_contour.png", draw_contour)
    # cv2.imshow("img", closing)
    # rospy.loginfo("finish analysis") 
    print "finish analysis"
    # rospy.loginfo("You can shut down this launch file now")
    print "You can shut down this launch file now"

def start_draw(stop_status):
    global draw_cnt
    global img_matrix
    # rospy.loginfo("waiting the vehicle to stop")
    print "waiting the vehicle to stop"
    if stop_status.data==True and draw_cnt==0:
        # rospy.loginfo("I begin to draw")
        print "I begin to draw"
        draw_now(img_matrix)
        draw_cnt=1
        img_analysis(img_matrix)
        # rospy.loginfo("finish drawing")
        print "finish drawing"
        pass

def pub_barrier_cnt(obj_cnt):
    pub = rospy.Publisher('barrier_cnt', Int32, queue_size=100)
    rate = rospy.Rate(1)
    # rospy.loginfo(contours_cnt)

    pub.publish(obj_cnt) 
    rate.sleep()



def update_coordinate(data2):
    global cat_pose
    global theta
    global obj_cnt
    cat_pose.x = data2.pose.pose.position.x
    cat_pose.y = data2.pose.pose.position.y
    theta = 2*acos(data2.pose.pose.orientation.w)
    pub_barrier_cnt(obj_cnt)


def main():
    # Initialize our node, which we will call task2
    rospy.init_node('draw_img', anonymous=True)
    # global init_time
    # global draw_cnt
    # global now_time
    # global img_matrix
    # init_time=rospy.Time.now()

    sub = rospy.Subscriber("catvehicle/front_laser_points", LaserScan, record_points)

    sub_stop_status = rospy.Subscriber("stop_status", Bool, start_draw)

    sub_odom = rospy.Subscriber("catvehicle/odom", Odometry, update_coordinate)
        
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

