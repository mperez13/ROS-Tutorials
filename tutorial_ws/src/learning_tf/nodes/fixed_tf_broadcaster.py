#!/usr/bin/env python   
import roslib 
roslib.load_manifest('learning_tf') 

import rospy 
import tf 


if __name__ == '__main__': 
    rospy.init_node('my_tf_broadcaster') 


    br = tf.TransformBroadcaster() 
    rate = rospy.Rate(10.0) 
    while not rospy.is_shutdown(): 
        # Create new transform from parent "turtle1" to new child "carrot1" 
        # carrot1 frame is 2 meters offset from the turtle1 frame.  
        br.sendTransform((0.0, 2.0, 0.0), 
                         (0.0, 0.0, 0.0, 1.0), 
                         rospy.Time.now(), 
                         "carrot1", 
                         "turtle1") 
        rate.sleep()
