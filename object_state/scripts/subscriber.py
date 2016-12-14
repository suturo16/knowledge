#!/usr/bin/env python
import roslib; roslib.load_manifest('json_prolog')

import rospy
from json_prolog import json_prolog
from suturo_perception_msgs.msg import ObjectDetection

def callback(currObj):
    prolog = json_prolog.Prolog()
    query = prolog.query("create_object_state_with_close('" +
                                        currObj.name+"', [" 
                                        + str(0.00) + "," + str(0.01) + "," + str(0.02) + "," + str(0.03) + ","
                                        + str(0.10) + "," + str(0.11) + "," + str(0.12) + "," + str(0.13) + ","
                                        + str(0.20) + "," + str(0.21) + "," + str(0.22) + "," + str(0.23) + ","
                                        + str(0.30) + "," + str(0.31) + "," + str(0.32) + "," + str(0.33) + "], " 
                                        + str(currObj.type) + ",'" + str(currObj.pose.header.frame_id) + "',"
                                        + str(currObj.width) + "," + str(currObj.height) + "," + str(currObj.depth) 
                                        + ", [" + str(currObj.pose.header.stamp) + "],  ObjInst)")
    for solution in query.solutions():
        rospy.loginfo('Found solution.')
    query.finish()
    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('perception_listener', anonymous=True)

    rospy.Subscriber("percepteros/object_detection", ObjectDetection, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
