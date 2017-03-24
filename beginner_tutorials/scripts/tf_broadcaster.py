#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import cv2.cv as cv
import numpy as np





class TfBroadcast():
    def __init__(self):
        self.br= tf.TransformBroadcaster()
        self.yaw=(math.pi/4) #Yaw angle of the rotated Transformation
        angle_quaternion =tf.transformations.quaternion_from_euler( 0, 0, self.yaw)
        r = rospy.Rate(100) #Value in Hz
        while not rospy.is_shutdown():
            #sendTransform(translation, rotation, time, child, parent)
            self.br.sendTransform((0,0,0), angle_quaternion ,rospy.get_rostime(),"/body_frame","/map")
            r.sleep()
        self.c
    
    def nothing(x):
        pass

if __name__ == "__main__":
    rospy.init_node('tf_broadcaster')
    try:
        TfBroadcast()
    except:
        rospy.logfatal("tf_broadcaster.py controller died")
        pass

