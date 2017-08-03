#!/usr/bin/env python  
import roslib
roslib.load_manifest('object_state')
import rospy

import tf
import rospy
import thread
from json_prolog import json_prolog

solutions = []

def start_querying():
	global solutions

	#invoke prolog, broadcaster
	prolog = json_prolog.Prolog()
	
	#do until killed (Ctrl-C)
	while(not rospy.is_shutdown()):
		query = prolog.query("get_tf_infos(Name,FrameID,Position,Orientation)")
		temp = []
		#loop for all solutions from get_tf_infos(N,F,P,O)
		for solution in query.solutions():
			temp.append(solution)
			#rospy.logerr(str(solution))
		query.finish()
		solutions = temp
		rospy.sleep(1)

def start_tf_publish():
	br = tf.TransformBroadcaster()

	while(not rospy.is_shutdown()):
		#loop for all solutions from get_tf_infos(N,F,P,O)
		for solution in solutions:
			#rospy.logerr("TEST")
		
			position = solution["Position"]
			orientation = solution["Orientation"] 
			
			#pushing the aquired information to tf
			br.sendTransform((float(position[0]), float(position[1]), float(position[2])),
                     (float(orientation[0]), float(orientation[1]), float(orientation[2]), float(orientation[3])),
                     rospy.Time.now(),
                     '/' + solution['Name'],
                     solution['FrameID'])
			rospy.sleep(0.01)


if __name__ == '__main__':
	#initialize the node 
	rospy.init_node('fluents_tf_broadcaster')
	rospy.wait_for_service('json_prolog/simple_query', timeout=10)

	#do the main work
	thread.start_new_thread(start_tf_publish, ())
	start_querying()

	#repeat until stopped
	rospy.spin()
