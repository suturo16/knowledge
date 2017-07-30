#!/usr/bin/env python  
import roslib
roslib.load_manifest('object_state')

import rospy
import tf
import thread
from json_prolog import json_prolog

if __name__ == '__main__':
    #init rosnode, prolog and tflistener
    rospy.init_node('test_connect_frames')
    rospy.wait_for_service('json_prolog/simple_query', timeout=10)
    listener = tf.TransformListener()
    rospy.sleep(0.25)

    prolog = json_prolog.Prolog()

    # Create the objects

    query = prolog.query("dummy_perception_with_close1(baum)")
    for solution in query.solutions():
        print('Call to dummy_perception worked')
    query.finish()
    print('----------------------------------------------------------')

    query = prolog.query("dummy_perception_with_close2(cakeSpatula)")
    for solution in query.solutions():
        print('Call to dummy_perception worked')
    query.finish()
    print('----------------------------------------------------------')

    # Get their respective infos
    query = prolog.query("get_tf_infos(Name,FrameID,Position,Orientation)")
    for solution in query.solutions():
        print(str(solution))
        if(solution["Name"] == "cakeSpatula1"):
            if(solution["FrameID"] != "/odom_combined"):
                rospy.logerr('FrameID after Connect Frames isnt right')
            if(solution["Position"][0] != 3 or solution["Position"][1] != -4 or solution["Position"][2] != 3):
                rospy.logerr('Position after Connect Frames isnt right')
    query.finish()

    print('----------------------------------------------------------')

    listener.waitForTransform('/odom_combined', '/cakeSpatula1', rospy.Time(0), rospy.Duration(4.5))
    (trans,rot) = listener.lookupTransform('/odom_combined', '/cakeSpatula1', rospy.Time(0))
    trans_list = list(trans)
    if(trans_list[0] != 3 or trans_list[1] != -4 or trans_list[2] != 3):
        rospy.logerr('Relative Position before connect frames isnt right')

    print('----------------------------------------------------------')

    rospy.sleep(2.25)
    query = prolog.query("test_connect_frames(baum1,cakeSpatula1)")
    for solution in query.solutions():
        print(str(solution))
    query.finish()
    print('----------------------------------------------------------')

    query = prolog.query("dummy_perception_with_close2(cakeSpatula)")
    for solution in query.solutions():
        print('Call to dummy_perception worked')
    query.finish()
    print('----------------------------------------------------------')

    query = prolog.query("get_tf_infos(Name,FrameID,Position,Orientation)")
    for solution in query.solutions():
        print(str(solution))
        if(solution["Name"] == "cakeSpatula1"):
            if(solution["FrameID"] != "/baum1"):
                rospy.logerr('FrameID after Connect Frames isnt right')
            if(solution["Position"][0] != 2 or solution["Position"][1] != -5 or solution["Position"][2] != 2):
                rospy.logerr('Position after Connect Frames isnt right')
    query.finish()
    print('----------------------------------------------------------')

    listener.waitForTransform('/odom_combined', '/cakeSpatula1', rospy.Time(0), rospy.Duration(4.5))
    (trans,rot) = listener.lookupTransform('/odom_combined', '/cakeSpatula1', rospy.Time(0))
    trans_list = list(trans)
    if(trans_list[0] != 3 or trans_list[1] != -4 or trans_list[2] != 3):
        rospy.logerr('Relative Position after connect frames isnt right')

    print('----------------------------------------------------------')

    rospy.sleep(2.25)
    query = prolog.query("test_disconnect_frames(baum1,cakeSpatula1)")
    for solution in query.solutions():
        print(str(solution))
    query.finish()
    print('----------------------------------------------------------')

    query = prolog.query("dummy_perception_with_close2(cakeSpatula)")
    for solution in query.solutions():
        print('Call to dummy_perception worked')
    query.finish()
    print('----------------------------------------------------------')

    query = prolog.query("get_tf_infos(Name,FrameID,Position,Orientation)")
    for solution in query.solutions():
        print(str(solution))
        if(solution["Name"] == "cakeSpatula1"):
            if(solution["FrameID"] != "/odom_combined"):
                rospy.logerr('FrameID after Connect Frames isnt right')
            if(solution["Position"][0] != 3 or solution["Position"][1] != -4 or solution["Position"][2] != 3):
                rospy.logerr('Position after Connect Frames isnt right')
    query.finish()
    print('----------------------------------------------------------')

    listener.waitForTransform('/odom_combined', '/cakeSpatula1', rospy.Time(0), rospy.Duration(4.5))
    (trans,rot) = listener.lookupTransform('/odom_combined', '/cakeSpatula1', rospy.Time(0))
    trans_list = list(trans)
    if(trans_list[0] != 3 or trans_list[1] != -4 or trans_list[2] != 3):
        rospy.logerr('Relative Position after disconnect frames isnt right')