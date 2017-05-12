#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Polygon, PolygonStamped, Point, Twist, Vector3

first_zero=0
first_zero_time=rospy.Time()
zero_time=rospy.Time()
just_start=1

def vel_callback(data):
    global first_zero
    global first_zero_time
    global zero_time
    global just_start
    pub = rospy.Publisher('stop_status', Bool, queue_size=100)
    # rate = rospy.Rate(1) # 10hz
    v=data.linear.x
    print v
    if just_start==1:
        
        if v<0.01 and v>-0.01:
            status=False
            rospy.loginfo(status)
            pub.publish(status)
        else:
            just_start=0
    else:
        if v<0.01 and v>-0.01:
            if first_zero==0:
                first_zero_time=rospy.Time.now()
                first_zero=1
                pass
            zero_time=rospy.Time.now()
            if zero_time - first_zero_time > rospy.Duration(5):

                status = True
                rospy.loginfo(status)
                pub.publish(status)
            
            else:
                status=False
                rospy.loginfo(status)
                pub.publish(status)
        else:
            first_zero=0
            status = False
            rospy.loginfo(status)
            pub.publish(status)
    # rate.sleep()


def stop_status():
    # Initialize our node, which we will call task2
    rospy.init_node('stop_status', anonymous=True)
    # Create the subscriber, subscribe to topic /catvehicle/front_laser_points
    sub = rospy.Subscriber("catvehicle/vel", Twist, vel_callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        stop_status()
    except rospy.ROSInterruptException:
        pass