import rospy
import numpy as np
from apriltags_ros.msg import * 
from camera_trigger import Camera
import tf

class ApriltagDetector:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.camera = Camera()

    def detect_tag(self, tag_frame='/april0_frame'):
        self.camera.trigger()
        self.tf_listener.waitForTransform('/world', tag_frame, rospy.Time(), rospy.Duration(4.0))
        (trans, q) = self.tf_listener.lookupTransform('/world', tag_frame, rospy.Time(0)) 
        T = tf.transformations.quaternion_matrix(q)
        T[:3,3]=np.array(trans)
        return T

if __name__=='__main__':
    rospy.init_node('apriltag_detector', anonymous=True) #initialize the node 
    ad = ApriltagDetector()
    T = ad.detect_tag()
    print(T)
