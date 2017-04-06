# Import the modules
import tf
import rospy

def get_transform(parent_frame,child_frame):
    rospy.init_node('connect_frames_node')
    listener = tf.TransformListener()
    rospy.sleep(0.25)
    now = rospy.Time.now()
    listener.waitForTransform(child_frame, parent_frame, now, rospy.Duration(4.5))
    (trans,rot) = listener.lookupTransform(child_frame, parent_frame, rospy.Time(0))
    return [list(trans), list(rot)]
