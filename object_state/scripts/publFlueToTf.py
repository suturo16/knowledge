 1 #!/usr/bin/env python  
 2 import roslib
 3 roslib.load_manifest('learning_tf')
 4 import rospy
 5 
 6 import tf
 7 import turtlesim.msg
 8 
 9 def handle_turtle_pose(msg, turtlename):
10     br = tf.TransformBroadcaster()
11     br.sendTransform((msg.x, msg.y, 0),
12                      tf.transformations.quaternion_from_euler(0, 0, msg.theta),
13                      rospy.Time.now(),
14                      turtlename,
15                      "world")
16 
17 if __name__ == '__main__':
18     rospy.init_node('turtle_tf_broadcaster')
19     turtlename = rospy.get_param('~turtle')
20     rospy.Subscriber('/%s/pose' % turtlename,
21                      turtlesim.msg.Pose,
22                      handle_turtle_pose,
23                      turtlename)
24     rospy.spin()