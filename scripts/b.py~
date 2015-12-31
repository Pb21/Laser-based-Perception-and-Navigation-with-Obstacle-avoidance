#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
from nav_msgs.msg import Odometry
import tf
import turtlesim.msg

#Broadcasting own co-ordinates
def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    q = []
    q.append(msg.pose.pose.orientation.x)
    q.append(msg.pose.pose.orientation.y)
    q.append(msg.pose.pose.orientation.z)
    q.append(msg.pose.pose.orientation.w)
    
    #Publishes it as a transform from frame "world" to frame "robotX".
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     q,
                     rospy.Time.now(),
                     turtlename,
                     "world")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/base_pose_ground_truth' % turtlename,
                     Odometry,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()

