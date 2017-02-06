#!/usr/bin/env python  
import roslib
roslib.load_manifest('object_state')
import rospy

import tf
#import objectdetection.msg

def startPublish():

	prolog = json_prolog.Prolog()
	br = tf.TransformBroadcaster()
	
	while(true)
		query = prolog.query("get_tf_infos(Name,ParentFrame,PositionList,OrientationList)")
	    for solutions in query.solutions():
	    	rospy.loginfo('At least I tried')
	    	br.sendTransform((solution[[ParentFrame[0]]],))
		query.finish()
		return objects

def handle_fluents_pose(msg, turtlename):
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "/odom_combined")

if __name__ == '__main__':
	#initialize the node 
    rospy.init_node('fluents_tf_broadcaster')
    startPublish()
    rospy.spin()
