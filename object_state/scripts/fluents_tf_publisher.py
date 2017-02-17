#!/usr/bin/env python  
import roslib
roslib.load_manifest('object_state')
import rospy

import tf
import rospy
from json_prolog import json_prolog

def startPublish():

	#invoke prolog, broadcaster
	prolog = json_prolog.Prolog()
	br = tf.TransformBroadcaster()
	
	#do until killed (Ctrl-C)
	while(True):
		query = prolog.query("get_tf_infos(Name,FrameID,Position,Orientation)")
		
		#loop for all solutions from get_tf_infos(N,F,P,O)
		for solution in query.solutions():
			
			position = solution["Position"]
			orientation = solution["Orientation"] 
			
			#pushing the aquired information to tf
			br.sendTransform((float(position[0]), float(position[1]), float(position[2])),
                     (float(orientation[0]), float(orientation[1]), float(orientation[2]), float(orientation[3])),
                     rospy.Time.now(),
                     '/' + solution['Name'],
                     solution['FrameID'])
		query.finish()
		rospy.sleep(0.1)


if __name__ == '__main__':
	#initialize the node 
	rospy.init_node('fluents_tf_broadcaster')
	rospy.wait_for_service('json_prolog/simple_query', timeout=10)

	#do the main work
	startPublish()

	#repeat until stopped
	rospy.spin()
