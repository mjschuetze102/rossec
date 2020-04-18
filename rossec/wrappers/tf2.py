# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#!/usr/bin/env python
import geometry_msgs.msg
from . import Publisher, Subscriber
from tf2_msgs.msg import TFMessage
import rospy
import rossec
from rossec.msg import SecuredMessage
import threading


class TransformBroadcaster:
    """
    Security wrapper for the TranformBroadcaster class
    """

    def __init__(self, queue_size=100):
        self.pub = rossec.Publisher("/encrypt/tf", TFMessage, queue_size=100)

    def sendTransform(self, transform):
        if not isinstance(transform, list):
            transform = [transform]
        self.pub.publish(TFMessage(transform))


class TransformListener():
    """
    Security wrapper for the TransformListener class
    """

    def __init__(self, buffer, queue_size=None, buff_size=65536, tcp_nodelay=False):
        self.buffer = buffer
        self.last_update = rospy.Time.now()
        self.last_update_lock = threading.Lock()
        self.tf_sub = rossec.Subscriber("/encrypt/tf", TFMessage, self.callback, queue_size=queue_size, buff_size=buff_size, tcp_nodelay=tcp_nodelay)
        self.tf_static_sub = rossec.Subscriber("/encrypt/tf_static", TFMessage, self.static_callback, queue_size=queue_size, buff_size=buff_size, tcp_nodelay=tcp_nodelay)

    def __del__(self):
        self.unregister()

    def unregister(self):
        """
        Unregisters all tf subscribers.
        """
        self.tf_sub.unregister()
        self.tf_static_sub.unregister()

    def check_for_reset(self):
        # Lock to prevent different threads racing on this test and update.
        # https://github.com/ros/geometry2/issues/341
        with self.last_update_lock:
            now = rospy.Time.now()
            if now < self.last_update:
                rospy.logwarn("Detected jump back in time of %fs. Clearing TF buffer." % (self.last_update - now).to_sec())
                self.buffer.clear()
            self.last_update = now

    def callback(self, data):
        self.check_for_reset()
        #who = data._connection_header.get('callerid', "default_authority")
        who = "default_authority"
        for transform in data.transforms:
            self.buffer.set_transform(transform, who)

    def static_callback(self, data):
        self.check_for_reset()
        #who = data._connection_header.get('callerid', "default_authority")
        who = "default_authority"
        for transform in data.transforms:
            self.buffer.set_transform_static(transform, who)

