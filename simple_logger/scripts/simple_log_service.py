#!/usr/bin/env python

from Queue import Queue
from suturo_knowledge_msgs.srv import *
import rospy
import rospkg
import time

queue = Queue()
top_level_names = Queue()
parent_to_child = dict()
ID = 0;

#data for the default experiment
default_experiment = {'creator' : 'CaterROS', 'description':'Pick and place'
,'experiment':'Two arm pick and place','experimentName':'exp-'+time.strftime("%Y-%m-%d-%H-%M"),'robot':'PR2'}

def handle_log_event(req):
	global ID
	ID = ID + 1
	#print "Logging action %s from startTime %i to endTime %i"%(req.nameOfAction, req.startTime.to_sec(), req.endTime.to_sec())
	queue.put((req, ID))
	if req.parentActionID == 0:
		top_level_names.put(str(ID)+'_'+req.nameOfAction)
	if req.parentActionID in parent_to_child:
		parent_to_child[req.parentActionID].extend((ID,req.nameOfAction))
	else:
		parent_to_child[req.parentActionID] = [(ID,req.nameOfAction)]
	return LogActionResponse(ID)

def handle_log_change_default(req):
	if req.creator:
		default_experiment['creator'] = req.creator
	if req.description:
		default_experiment['description'] = req.description
	if req.experiment:
		default_experiment['experiment'] = req.experiment
	if req.experimentName:
		default_experiment['experimentName'] = req.experimentName
	if req.robot:
		default_experiment['robot'] = req.robot
	return LogExperimentDescriptionResponse(True)

def write_on_shutdown():
	timestamps = []
	rospack = rospkg.RosPack()
	# Get the output/input paths
	package_path = rospack.get_path('simple_logger')
	timestr = time.strftime("%d%m%y_%H%M%S")
	output = ""
	for line in open(package_path+"/owl/log_header.owl"):
		output += line
	#Write every action into our owl file
	while not queue.empty():
		out = queue.get()
		#print "Logging action %s from startTime %i to endTime %i"%(out[0].nameOfAction, out[0].startTime.to_sec(), out[0].endTime.to_sec())
		output += '\n    <owl:NamedIndividual rdf:about="&suturolog;'+str(out[1])+'_'+out[0].nameOfAction+'">\n'
		output += '        <rdf:type rdf:resource="&suturolog;'+out[0].nameOfAction+'"/>\n'
		output += '        <knowrob:taskSuccess rdf:datatype="&xsd;boolean">'+str(out[0].success)+'</knowrob:taskSuccess>\n'
		timestamps.append(out[0].startTime.data.to_time())
		output += '        <knowrob:startTime rdf:resource="&suturolog;timepoint_'+str(out[0].startTime.data)+'"/>\n'
		timestamps.append(out[0].endTime.data.to_time())
		output += '        <knowrob:endTime rdf:resource="&suturolog;timepoint_'+str(out[0].endTime.data)+'"/>\n'
		if out[1] in parent_to_child:
			for action_tupel in parent_to_child[out[1]]:
				output += '        <knowrob:subAction rdf:resource="&suturolog;'+str(action_tupel[0])+'_'+action_tupel[1]+'"/>\n'
		if out[0].parameters:
			for para in out[0].parameters:
				output += '        <owl:equivalentClass>\n'
				output += '            <owl:Restriction>\n'
				output += '                <owl:onProperty rdf:resource="&suturolog;parameter"/>\n'
				output += '                <owl:hasValue rdf:resource="&xsd;string">'+para+'</owl:hasValue>\n'
				output += '             </owl:Restriction>\n'
				output += '		</owl:equivalentClass>\n'
		output += '        <knowrob:robot rdf:datatype="&xsd;string">PR2</knowrob:robot>\n'
		output += '    </owl:NamedIndividual>\n'
	# Write the timestamps
	sortedTimes = sorted(timestamps)
	for times in sortedTimes:
		print type(times)
		output += '\n    <owl:NamedIndividual rdf:about="&suturolog;timepoint_'+str(times)+'">\n'
		output += '        <rdf:type rdf:resource="&knowrob;TimePoint"/>\n'
		output += '        <knowrob:robot rdf:datatype="&xsd;string">PR2</knowrob:robot>\n'
		output += '    </owl:NamedIndividual>\n'
	# write the experiment tag
	output += '\n    <owl:NamedIndividual rdf:about="&suturolog;RobotExperiment_asdfeddssd">\n'
	output += '        <rdf:type rdf:resource="&knowrob;RobotExperiment"/>\n'
	while not queue.empty():
		top_level_action = queue.get()
		output += '	       <knowrob:subAction rdf:resource="&suturolog;'+top_level_action+'"/>\n'
	output += '        <knowrob:creator rdf:datatype="&xsd;string">'+default_experiment['creator']+'</knowrob:creator>\n'
	output += '        <knowrob:description rdf:datatype="&xsd;string">'+default_experiment['description']+'</knowrob:description>\n'
	output += '        <knowrob:experiment rdf:datatype="&xsd;string">'+default_experiment['experiment']+'</knowrob:experiment>\n'
	output += '        <knowrob:experimentName rdf:datatype="&xsd;string">'+default_experiment['experimentName']+'</knowrob:experimentName>\n'
	output += '        <knowrob:robot rdf:datatype="&xsd;string">'+default_experiment['robot']+'</knowrob:robot>\n'
	output += '        <knowrob:startTime rdf:resource="&suturolog;timepoint_'+str(sortedTimes[0])+'"/>\n'
	output += '        <knowrob:endTime rdf:resource="&suturolog;timepoint_'+str(sortedTimes[-1])+'"/>\n'
	output += '    </owl:NamedIndividual>\n'
	output += '\n</rdf:RDF>'
	with open('suturo_log_'+timestr+'.owl','w') as log_file:
		log_file.write(output)


def log_event_server():
    rospy.init_node('log_event_server')
    s = rospy.Service('log_event', LogAction, handle_log_event)
    s2 = rospy.Service('log_event_experiment_description', LogExperimentDescription, handle_log_change_default)
    print "Ready to log actions and subactions."
    rospy.on_shutdown(write_on_shutdown)
    rospy.spin()

if __name__ == "__main__":
    log_event_server()
