#!/usr/bin/env python

#author: Tianyang Chen, Yawei Ding, Yuanzhengyu Li

import rospy
import math
from math import sin, cos

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

def callback(data):
    #init some important parameters
    view=[]

    raw_data=data.ranges
    x=[]
    y=[]
    # r = []
    # count = 0
    # angular = []
    # sigma = 0.25 #this parameter denotes the sensitivity of the sensor
    # obj_num = 1
    # obj_num2 = 0
    # seq = 0;
    num_points = len(data.ranges)
    # r_index_divide = []
    pi = 3.1415926
    for i in range(num_points):
        x.append(raw_data[i]*sin(i*pi/num_points)+2.4)
        y.append(raw_data[i]*cos(i*pi/num_points))
        pass
    for i in range(num_points):
        if y[i]>-0.5 and y[i]<0.5:
            view.append(x[i])
            pass
        pass
    # for x in raw_data:
    #     pass
    # for i in range(30,150):
    #     view.append(raw_data[i])
    #     pass

    pub = rospy.Publisher('catvehicle/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    msg=Twist()
    v_linear=Vector3()
    v_angular=Vector3()


    if min(view)<2:
        v_linear.x=-3
        v_linear.y=0
        v_linear.z=0
        v_angular.x=0
        v_angular.y=0
        v_angular.z=0
        msg.linear=v_linear
        msg.angular=v_angular
        rospy.loginfo(msg)
        pub.publish(msg)
    elif min(view)<4:
        v_linear.x=1
        v_linear.y=0
        v_linear.z=0
        v_angular.x=0
        v_angular.y=0
        v_angular.z=-0.8
        msg.linear=v_linear
        msg.angular=v_angular
        rospy.loginfo(msg)
        pub.publish(msg)
    else:
        v_linear.x=2
        v_linear.y=0
        v_linear.z=0
        v_angular.x=0
        v_angular.y=0
        v_angular.z=0
        msg.linear=v_linear
        msg.angular=v_angular
        rospy.loginfo(msg)
        pub.publish(msg)
    rate.sleep()

    # #filt all the points within 10 meters of the sensor
    # rospy.loginfo(data)
    # for x in data.ranges:
    #     count += 1
    #     if x < 10:
    #         r.append(x)
    #         angular.append(count)
    #         pass
    #     pass

    # #this algorithm combine some adjacent points to the same object
    # for r_index in range(len(r)-1):
    #     if r[r_index] - r[r_index+1] < -sigma or r[r_index] - r[r_index+1] > sigma:
    #         obj_num += 1
    #         pass
    #     pass

    # #divide our laser data into different objects
    # r_divide = [[] for i in range(obj_num)]
    # angular_divide = [[] for i in range(obj_num)]
    # r_divide[0].append(r[0])
    # angular_divide[0].append(angular[0])
    # for r_index in range(len(r)-1):
    #     if r[r_index] - r[r_index+1] >= -sigma and r[r_index] - r[r_index+1] <= sigma:
    #         r_divide[obj_num2].append(r[r_index+1])
    #         angular_divide[obj_num2].append(angular[r_index+1])
    #     else:
    #         obj_num2 += 1
    #         r_divide[obj_num2].append(r[r_index+1])
    #         angular_divide[obj_num2].append(angular[r_index+1])

    # #publish all the polygons
    # for i in range(obj_num):
    #     msg = PolygonStamped();
    #     detection = Polygon();
    #     t1 = rospy.Time.now();
    #     # puts a time stamp at t1 - as the initial point
    #     msg.header.stamp = t1;
    #     seq += 1;
    #     msg.header.seq = seq;
    #     # catvehicle/odom is the odometry topic you are subscribing to 
    #     msg.header.frame_id = 'catvehicle/odom'
    #     x = []; 
    #     y = []; 
    #     z = 0.1;
    #     #convert the polar coordinates to Catesian coordinates
    #     for j in range(len(r_divide[i])):
    #         x.append(r_divide[i][j]*sin(angular_divide[i][j]*pi/num_points)+2.4)
    #         y.append(-r_divide[i][j]*cos(angular_divide[i][j]*pi/num_points))
    #         pass
    #     # here we use the bounding box strategy to represent the barrier 
    #     detection.points.append(Point(min(x),min(y),z));
    #     detection.points.append(Point(max(x),min(y),z));
    #     detection.points.append(Point(max(x),max(y),z));
    #     detection.points.append(Point(min(x),max(y),z));
    #     # then this polygon is then converted into a polgon message
    #     msg.polygon = detection;
    #     # writing loginto into stdout
    #     rospy.loginfo(msg);
    #     # publishing the ros message with the polygon information
    #     pub = rospy.Publisher('detections', PolygonStamped, queue_size=10)
    #     rate = rospy.Rate(120)
    #     pub.publish(msg)
    #     rate.sleep()
    #     pass





def easy_run():
    # Initialize our node, which we will call task2
    rospy.init_node('easy_run', anonymous=True)
    # Create the subscriber, subscribe to topic /catvehicle/front_laser_points
    sub = rospy.Subscriber("catvehicle/front_laser_points", LaserScan, callback)
    
    rospy.spin()

if __name__ == '__main__':
    easy_run()
    # try:
    #     main()
    # except rospy.ROSInterruptException:
    #     pass

