# Import the modules
import tf
import rospy

def get_transform(parent_frame,child_frame):
    rospy.loginfo('Send query')
    rospy.init_node('connect_frames_node')
    listener = tf.TransformListener()
    rospy.sleep(0.25)
    listener.waitForTransform(parent_frame, child_frame, rospy.Time(0), rospy.Duration(4.5))
    (trans,rot) = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
    return [list(trans), list(rot)]
