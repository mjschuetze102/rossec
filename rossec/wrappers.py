#!/usr/bin/env python
import rospy
from rossec.msg import String


class Publisher:
    """
    Security wrapper for the Publisher class
    """

    def __init__(self, *args, **kwargs):
        data_class = args[1]
        print("Data Class: ", data_class)
        args = args[:1] + (String,) + args[2:]
        print("New Data Class: ", args[1])
        self.pub = rospy.Publisher(*args, **kwargs)

    def publish(self, *args, **kwargs):
        self.pub.publish(*args, **kwargs)

class Subscriber:
    """
    Security wrapper for the Subscriber class
    """

    def __init__(self, *args, **kwargs):
        data_class = args[1]
        callback = args[2] if len(args) > 2 else kwargs.get('callback', None)
        callback_args = args[3] if len(args) > 3 else kwargs.get('callback_args', None)
        print("Data Class: ", data_class)
        print("Callback: ", callback)
        print("Callback Args: ", callback_args)
        args = args[:1] + (String,) + args[2:]
        print("New Data Class: ", args[1])
        self.sub = rospy.Subscriber(*args, **kwargs)

    def unregister(self):
        self.sub.unregister()

