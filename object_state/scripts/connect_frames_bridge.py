#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
"""
Small help service to calculate the position of the childframe
in the parent frame
"""
from suturo_knowledge_msgs.srv import *
from geometry_msgs.msg import PoseStamped
import tf
import rospy
from json_prolog import json_prolog

optimusPrime = None

# This method gets the String from our SRV File and uses the Text to Speech from Pepper
# to say stuff. Until now we will only return Complete.
# We are using TF to transform the point we get to Torso frame from Pepper.
def connect_frames_service(req):
    # get the frames
    parent_frame = req.parentFrame
    child_frame = req.childFrame

    # lockup the transform
    (trans,rot) = optimusPrime.lookupTransform(child_frame, parent_frame, rospy.Time(0))

    # Invoke Prolog
    print(parent_frame)
    prolog = json_prolog.Prolog()
    query = prolog.query("connect_frames('"+parent_frame+"','"+child_frame+
        "',[ [ "+str(trans[0])+","+str(trans[1])+","+str(trans[2])+"], ["
        +str(rot[0])+","+str(rot[1])+","+str(rot[2])+","+str(rot[3])+" ] ])")
    for solution in query.solutions():
        print str(solution)
    query.finish()

    return ConnectFramesResponse(True)


# We are using the NAO API from Aldebaran, we start the session with the given IP address. And save the TTS as
# a global variable in this script. That is the best way of getting it inside the Say Method.
if __name__ == "__main__":
    rospy.init_node('connect_frames_node')
    # initializing our transformer!
    optimusPrime = tf.TransformListener()
    s = rospy.Service('connect_frames_service', ConnectFrames, connect_frames_service)
    rospy.spin()