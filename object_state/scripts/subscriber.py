#!/usr/bin/env python
import roslib; roslib.load_manifest('json_prolog')

import rospy
from json_prolog import json_prolog
from suturo_perception_msgs.msg import ObjectDetection

def callback(currObj):
    prolog = json_prolog.Prolog()
    query = prolog.query("create_object_state_with_close('" 
                            + currObj.name+"', [ ["
                            + str(currObj.pose.pose.position.x) + "," 
                            + str(currObj.pose.pose.position.y) + "," 
                            + str(currObj.pose.pose.position.z) + "],["
                            + str(currObj.pose.pose.orientation.x) + "," 
                            + str(currObj.pose.pose.orientation.y) + "," 
                            + str(currObj.pose.pose.orientation.z) + "," 
                            + str(currObj.pose.pose.orientation.w) + "] ], " 
                            + str(currObj.type) + ",'" + str(currObj.pose.header.frame_id) + "',"
                            + str(currObj.width) + "," + str(currObj.height) + "," + str(currObj.depth) + ", [" 
                            + str(currObj.pose.header.stamp)+ "],  ObjInst)")
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
    rospy.wait_for_service('json_prolog/simple_query', timeout=10)

    
    rospy.Subscriber("percepteros/object_detection", ObjectDetection, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
