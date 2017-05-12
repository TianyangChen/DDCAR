#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Polygon, PolygonStamped, Point, Twist, Vector3




def stop_after_some_time():
    # Initialize our node, which we will call task2

    pub = rospy.Publisher('doIt', Bool, queue_size=100)
    rospy.init_node('stop_after_some_time', anonymous=True)
    i=0
    while 1:
        i+=1
        if i<=130:
            status=False
            pub.publish(status)
            rospy.sleep(1.)
            pass
        else:
            status=True
            pub.publish(status)
            rospy.sleep(1.)
        pass


if __name__ == '__main__':
    try:
        stop_after_some_time()
    except rospy.ROSInterruptException:
        pass