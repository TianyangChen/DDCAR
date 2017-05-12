#!/usr/bin/env python

#author: Tianyang Chen, Yawei Ding, Yuanzhengyu Li

import rospy
import math
from math import sin, cos, sqrt

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Polygon, PolygonStamped, Point, Twist, Vector3
from nav_msgs.msg import Odometry

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
    if only_once==0:
        init_time=rospy.Time.now()
        only_once=1
        pass
    now_time = rospy.Time.now()
    #init some important parameters
    view=[]

    raw_data=data.ranges
    x=[]
    y=[]

    num_points = len(data.ranges)
    # r_index_divide = []
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

    pub = rospy.Publisher('catvehicle/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    msg=Twist()
    v_linear=Vector3()
    v_angular=Vector3()
    # print init_time
    # print now_time
    # print rospy.Duration(15)
    if sqrt(cat_pose.x * cat_pose.x + cat_pose.y * cat_pose.y) < 60 and now_time - init_time < rospy.Duration(20):
        print min(view)
        if min(view)<5:
            v_linear.x=-3
            v_linear.y=0
            v_linear.z=0
            v_angular.x=0
            v_angular.y=0
            v_angular.z=0
            msg.linear=v_linear
            msg.angular=v_angular
            #rospy.loginfo(msg)
            print "back"
            pub.publish(msg)
        elif min(view)<20:
            v_linear.x=3
            v_linear.y=0
            v_linear.z=0
            v_angular.x=0
            v_angular.y=0
            v_angular.z=1
            msg.linear=v_linear
            msg.angular=v_angular
            #rospy.loginfo(msg)
            print "turn left"
            pub.publish(msg)
        else:
            v_linear.x=5
            v_linear.y=0
            v_linear.z=0
            v_angular.x=0
            v_angular.y=0
            v_angular.z=0
            msg.linear=v_linear
            msg.angular=v_angular
            #rospy.loginfo(msg)
            print "go straight"
            pub.publish(msg)
        pass
    else:
        v_linear.x=0
        v_linear.y=0
        v_linear.z=0
        v_angular.x=0
        v_angular.y=0
        v_angular.z=0
        msg.linear=v_linear
        msg.angular=v_angular
        print "stop"
        #rospy.loginfo(msg)
        pub.publish(msg)

    rate.sleep()


def callback2(data2):
    global cat_pose
    cat_pose.x = data2.pose.pose.position.x
    cat_pose.y = data2.pose.pose.position.y



def easy_run():

    # Initialize our node, which we will call task2
    rospy.init_node('easy_run', anonymous=True)
    global now_time
    global init_time
    
    # Create the subscriber, subscribe to topic /catvehicle/front_laser_points
    sub = rospy.Subscriber("catvehicle/front_laser_points", LaserScan, callback)
    
    sub2 = rospy.Subscriber("catvehicle/odom", Odometry, callback2)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        easy_run()
    except rospy.ROSInterruptException:
        pass

