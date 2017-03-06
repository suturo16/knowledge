#!/usr/bin/env python
import roslib; roslib.load_manifest('json_prolog')

import rospy
from json_prolog import json_prolog
from suturo_perception_msgs.msg import ObjectDetection


object_map = {}
last_update_map = {}
prolog = json_prolog.Prolog()
# Timeout of prolog update in nanosekunden (50.000.000 sind also 0.05 sekunden, 66.666.666 ungefaehr 15hz)
TIMEOUT = 65666666
# Distance of the position and orientation. 
POSE_DISTANCE = 0.01
QUAT_DISTANCE = 0.01

def callback(currObj):

    if ((not currObj.name in object_map) 
        or (not isNear(currObj, object_map[currObj.name])) 
        or (last_update_map[currObj.name].to_nsec() < rospy.Time.now().to_nsec() - TIMEOUT)):
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
        rospy.loginfo('Send query')
        #for solution in query.solutions():
        #   rospy.loginfo('Found solution.')
        query.finish()
        object_map[currObj.name] = currObj
        last_update_map[currObj.name] = rospy.Time.now()


def isNear(currObj, oldObj):
    return ((abs(currObj.pose.pose.position.x - oldObj.pose.pose.position.x) <= (oldObj.pose.pose.position.x*POSE_DISTANCE)) 
        and (abs(currObj.pose.pose.position.y - oldObj.pose.pose.position.y) <= (oldObj.pose.pose.position.y*POSE_DISTANCE)) 
        and (abs(currObj.pose.pose.position.z - oldObj.pose.pose.position.z) <= (oldObj.pose.pose.position.z*POSE_DISTANCE)) 
        and (abs(currObj.pose.pose.orientation.x - oldObj.pose.pose.orientation.x) <= (oldObj.pose.pose.orientation.x*QUAT_DISTANCE)) 
        and (abs(currObj.pose.pose.orientation.y - oldObj.pose.pose.orientation.y) <= (oldObj.pose.pose.orientation.y*QUAT_DISTANCE)) 
        and (abs(currObj.pose.pose.orientation.z - oldObj.pose.pose.orientation.z) <= (oldObj.pose.pose.orientation.z*QUAT_DISTANCE)) 
        and (abs(currObj.pose.pose.orientation.w - oldObj.pose.pose.orientation.w) <= (oldObj.pose.pose.orientation.w*QUAT_DISTANCE)))
    
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
