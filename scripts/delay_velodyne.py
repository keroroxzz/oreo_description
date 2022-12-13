#!/usr/bin/env python3

import functools
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2

rospy.init_node("delay")

pub = rospy.Publisher("/velodyne_points_delayed", PointCloud2, queue_size=4)

def delayed_callback(msg, event):
    msg.header.stamp += rospy.Duration(1.0)
    pub.publish(msg)

def callback(msg):
    timer = rospy.Timer(rospy.Duration(1.0),
                        functools.partial(delayed_callback, msg),
                        oneshot=True)

sub = rospy.Subscriber("/velodyne_points", PointCloud2, callback, queue_size=4)

rospy.spin()