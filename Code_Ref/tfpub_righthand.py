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

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

def neuron_tfpub():
    pub = rospy.Publisher('neuron_tf_righthand', PoseStamped, queue_size=10)
    rospy.init_node('neuron_tfpub_righthand', anonymous=True)
    rate = rospy.Rate(200) # 10hz
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        br.sendTransform((0,0,0),(0,0,0.70710678,0.70710678),rospy.Time.now(),"RightHand_adjust","RightHand")
        try:
            (trans, rot) = listener.lookupTransform('/Hips','/RightHand_adjust',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        neuron_hand_pose = PoseStamped()
        neuron_hand_pose.header = Header()
        neuron_hand_pose.header.stamp = now
        neuron_hand_pose.header.frame_id = "map_right"
        neuron_hand_pose.pose.position.x = trans[0]
        neuron_hand_pose.pose.position.y = trans[1]
        neuron_hand_pose.pose.position.z = trans[2]
        neuron_hand_pose.pose.orientation.x = rot[0]
        neuron_hand_pose.pose.orientation.y = rot[1]
        neuron_hand_pose.pose.orientation.z = rot[2]
        neuron_hand_pose.pose.orientation.w = rot[3]
        rospy.loginfo(neuron_hand_pose)
        pub.publish(neuron_hand_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        neuron_tfpub()
    except rospy.ROSInterruptException:
        pass
