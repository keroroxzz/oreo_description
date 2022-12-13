#! /usr/bin/env python3

import rospy
import numpy as np
import math
import cv2

import tf2_ros
import tf.transformations as tr

from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, PointCloud, ChannelFloat32
from cv_bridge import CvBridge
cvBridge = CvBridge()


pc = PointCloud()
pc.channels.append(ChannelFloat32(name='weight'))
pc.header.frame_id = 'map'

uvmap = np.zeros((0,0))

def bakeUVmap(shape, fov=72.0):

    focal = shape[1]/2.0/math.tan(fov*math.pi/360.0)

    v, u = np.mgrid[0:shape[0], 0:shape[1]]
    
    u = -(u-shape[1]/2.0)
    v = -(v-shape[0]/2.0)
    w = np.ones(shape[:2])*focal

    vec = np.stack([w,u,v], axis=-1).reshape(shape[0], shape[1], 3)
    vec /= np.linalg.norm(vec, axis=2, keepdims=True)
    vec /= vec[:,:,0][:,:,np.newaxis]
    vec = vec.reshape(-1,3)
    vec = np.concatenate((vec, np.ones((vec.shape[0], 1), dtype=vec.dtype)), axis=1)

    return vec

def transformToMatrix(t):
    t = t.transform
    p = np.array([t.translation.x, t.translation.y, t.translation.z])
    q = np.array([t.rotation.x, t.rotation.y,
                  t.rotation.z, t.rotation.w])

    q /= np.linalg.norm(q)
    m = tr.quaternion_matrix(q)
    m[0:3, -1] = p
    return m

def callback(msg):
    global cvBridge, uvmap, map_pub, tfbuf, voxel

    depth = cvBridge.imgmsg_to_cv2(msg)
    flatdepth = depth.reshape(-1,1)

    # initialize the uvmap with the corresponding size
    if flatdepth.shape[0] != uvmap.shape[0]:
        uvmap = bakeUVmap(depth.shape, 72.0)

    # get transformation matrix
    transform = tfbuf.lookup_transform('map', 'robot', msg.header.stamp, rospy.Duration(4.0))
    m = transformToMatrix(transform)

    d = uvmap.copy()
    d[:,:3] *= flatdepth
    d = np.einsum("ij,kj->ki", m, d)

    mena_position = np.mean(d[:,:3], axis=0, keepdims=True)
    norm_position = d[:,:3]-mena_position
    radius = np.linalg.norm(norm_position, axis=1)
    mean_radius = np.mean(radius)
    std_radius = np.std(radius)
    norm_radius = (radius-mean_radius)/std_radius
    weight = np.clip(2.0-norm_radius, 0.0, 1.0)
    d[:,3] = weight

    data = Float32MultiArray()
    data.layout.data_offset=4
    data.data = d.reshape(-1)
    pcf_pub.publish(data)

    pc.header.stamp = rospy.Time.now()
    pc.points = []
    pc.channels[0].values = weight
    for x in range(d.shape[0]):
            pc.points.append(Point(d[x,0], d[x,1], d[x,2]))
    
    pc_pub.publish(pc)
    cv2.imshow("depth",depth/np.max(depth))
    cv2.waitKey(1)

if __name__ == "__main__":
    global pc_pub, pcf_pub, tfbuf, tflistener

    rospy.init_node("Range2PointCloud")
    
    # tf initialization
    tfbuf = tf2_ros.Buffer(rospy.Duration(2.0))
    tflistener = tf2_ros.TransformListener(tfbuf)

    pc_pub = rospy.Publisher("PointCloud", PointCloud, queue_size=1)
    pcf_pub = rospy.Publisher("PointCloud_float", Float32MultiArray, queue_size=1)
    rospy.Subscriber("depth", Image, callback, queue_size=1)
    
    rospy.spin()