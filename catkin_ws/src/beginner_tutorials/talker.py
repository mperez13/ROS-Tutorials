#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

# rospy - if you are writing a ROS Node
import rospy

# std_msgs.msg - so we can reuse std_msgs/String message type (a simple 
# string container) for publishing
from std_msgs.msg import String

def talker():
    # declares that your node is publishing to the chatter topic using 
    #   the message type String	
    # queue_size - limits amount of queued messages if any subscriber 
    #    is not receiving them fast enough.
    pub = rospy.Publisher('chatter', String, queue_size=10)

    #rospy.init_node(NAME) - tells rospy the name of your code; needs 
    #    this info to start communicationw/ ROS Master
    rospy.init_node('talker', anonymous=True)
    
    #create rate object; w/ help of sleep(), offers way for looping at 
    #    desired rate
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        # rospy.logininfo(str) - performs: the messages get printed to 
        #   screen, it gets written to the Node's log file, and it gets 
        #   written to rosout.
        rospy.loginfo(hello_str)

        #pub.publish(hello_str) - publishes a string to our chatter topic
        pub.publish(hello_str)

        #loop calls r.sleep(), which sleeps just long enough to maintain 
        #   desired rate through loop
        rate.sleep()

# catches rospy.ROSInterruptException exception, which can be thrown 
#    by rospy.sleep() and rospy.Rate.sleep() methods when Ctrl-C is 
#    pressed or your Node is otherwise shutdown
# Exception is raised is so that you don't accidentally continue 
#    executing code after the sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass












